#include "NetworkRadio.hpp"
#include <Utils.hpp>
#include "Geometry2d/Util.hpp"
#include "status.h"

using namespace boost::asio;
using ip::udp;

NetworkRadio::NetworkRadio(int server_port, int robot_port)
    : _socket(_context, udp::endpoint(udp::v4(), server_port)),
      _server_port{server_port},
      _robot_port{robot_port} {}

NetworkRadio::~NetworkRadio() {}

bool NetworkRadio::open() {
    startReceive();
    return true;
}

void NetworkRadio::startReceive() {
    // Set a receive callback
    _socket.async_receive_from(
        boost::asio::buffer(_recv_buffer), _robot_endpoint,
        [this] (const boost::system::error_code& error, std::size_t num_bytes) {
            receivePacket(error, num_bytes);
        });
}

bool NetworkRadio::isOpen() const {
    return _socket.is_open();
}

void NetworkRadio::send(Packet::RadioTx& packet) {
    // Get a list of all the IP addresses this packet needs to be sent to
    for (int robot_idx = 0; robot_idx < packet.robots_size(); robot_idx++) {
        const Packet::Control& control = packet.robots(robot_idx).control();
        uint32_t robot_id = packet.robots(robot_idx).uid();

        // Build the control packet for this robot.
        std::array<uint8_t, rtp::HeaderSize + sizeof(rtp::RobotTxMessage)> forward_packet;

        rtp::Header* header = reinterpret_cast<rtp::Header*>(&forward_packet[0]);
        header->port = rtp::PortType::CONTROL;
        header->address = rtp::BROADCAST_ADDRESS;
        header->type = rtp::MessageType::CONTROL;

        size_t offset = sizeof(rtp::Header);
        rtp::RobotTxMessage* msg = reinterpret_cast<rtp::RobotTxMessage*>(
                &forward_packet[offset]);

        // Set fields of the control message.
        msg->uid = robot_id;

        msg->message.controlMessage.bodyX = static_cast<int16_t>(
            control.xvelocity() * rtp::ControlMessage::VELOCITY_SCALE_FACTOR);
        msg->message.controlMessage.bodyY = static_cast<int16_t>(
            control.yvelocity() * rtp::ControlMessage::VELOCITY_SCALE_FACTOR);
        msg->message.controlMessage.bodyW = static_cast<int16_t>(
            control.avelocity() * rtp::ControlMessage::VELOCITY_SCALE_FACTOR);
        msg->message.controlMessage.dribbler =
            clamp(static_cast<uint16_t>(control.dvelocity()) * 2, 0, 255);

        msg->message.controlMessage.shootMode = control.shootmode();
        msg->message.controlMessage.kickStrength = control.kcstrength();
        msg->message.controlMessage.triggerMode = control.triggermode();
        msg->message.controlMessage.song = control.song();

        // Fetch the IP address
        auto range = _robot_ip_map.left.equal_range(robot_id);

        // Loop over all addresses for this robot; we want to send multiple copies.
        for (auto it = range.first; it != range.second; it++) {
            // Send to the given IP address
            ip::address robot_ip = it->second;
            udp::endpoint _robot_endpoint(robot_ip, _robot_port);
            _socket.async_send_to(boost::asio::buffer(forward_packet), _robot_endpoint,
                [] (const boost::system::error_code& error, std::size_t num_bytes) {
                    // Handle errors.
                    if (error) {
                        std::cerr << "Error sending: "
                            << error << " in " __FILE__ << std::endl;
                    }
                });
        }
    }
}

void NetworkRadio::receive() {
    // Let boost::asio handles callbacks
    _context.poll();
}

void NetworkRadio::receivePacket(
        const boost::system::error_code& error, std::size_t num_bytes) {
    if (error) {
        std::cerr << "Error receiving: " << error << " in " __FILE__ << std::endl;
        return;
    } else if (num_bytes != rtp::ReverseSize) {
        std::cerr << "Invalid packet length: expected "
            << rtp::ReverseSize << ", got " << num_bytes << std::endl;
        return;
    }

    rtp::RobotStatusMessage* msg = reinterpret_cast<rtp::RobotStatusMessage*>(
            &_recv_buffer[sizeof(rtp::Header)]);

    Packet::RadioRx packet;
    packet.set_timestamp(RJ::timestamp());
    packet.set_robot_id(msg->uid);

    // Find out which robot this corresponds to.
    auto ip_iter = _robot_ip_map.right.find(_robot_endpoint.address());

    int robot_id = msg->uid;

    // We already have this robot registered; this is a reverse packet
    if (ip_iter == _robot_ip_map.right.end()) {
        // This is a new robot, so we need to register this robot's UID
        registerRobot(msg->uid, _robot_endpoint.address());
    } else if (ip_iter->second != robot_id) {
        // This robot has been reassigned. Re-registering it sets the IP->ID
        // map correctly, but we still need to remove it from the ID->IP map.
        std::cerr << "Warning: UID of robot assigned IP "
            << ip_iter->first << " changed to " << ip_iter->second << ". Reassigning.";
        auto matching_id = _robot_ip_map.left.equal_range(ip_iter->second);
        auto matching_ip = [&] (const RobotIpMap::left_map::value_type& v) {
            return v.second == ip_iter->first;
        };

        RobotIpMap::left_map::iterator mid_begin = matching_id.first,
                                       mid_end = matching_id.second;

        // auto to_remove = std::remove_if(mid_begin, mid_end, matching_ip);
        // _robot_ip_map.left.erase(to_remove, _robot_ip_map.left.end());
        registerRobot(msg->uid, _robot_endpoint.address());
    }

    packet.set_hardware_version(Packet::RJ2015);
    packet.set_battery(msg->battVoltage *
            rtp::RobotStatusMessage::BATTERY_SCALE_FACTOR);

    if (Packet::BallSenseStatus_IsValid(msg->ballSenseStatus)) {
        packet.set_ball_sense_status(
                Packet::BallSenseStatus(msg->ballSenseStatus));
    }

    // Using same flags as 2011 robot. See common/status.h
    // Report that everything is good b/c the bot currently has no way of
    // detecting kicker issues
    packet.set_kicker_status((msg->kickStatus ? Kicker_Charged : 0) |
                             (msg->kickHealthy ? Kicker_Enabled : 0) |
                             Kicker_I2C_OK);

    // Motor errors
    for (int i = 0; i < 5; i++) {
        bool err = msg->motorErrors & (1 << i);
        packet.add_motor_status(err ? Packet::MotorStatus::Hall_Failure
                                    : Packet::MotorStatus::Good);
    }

    for (std::size_t i = 0; i < 4; i++) {
        packet.add_encoders(msg->encDeltas[i]);
    }

    // fpga status
    if (Packet::FpgaStatus_IsValid(msg->fpgaStatus)) {
        packet.set_fpga_status(Packet::FpgaStatus(msg->fpgaStatus));
    }

    {
        // Add reverse packets
        std::lock_guard<std::mutex> lock(_reverse_packets_mutex);
        _reversePackets.push_back(packet);
    }

    // Restart receiving
    startReceive();
}

void NetworkRadio::switchTeam(bool) {}

void NetworkRadio::registerRobot(int robot, ip::address ip) {
    // Insert the robot into the bimap, going both directions
    _robot_ip_map.right.insert(RobotIpMap::right_map::value_type(ip, robot));
    _robot_ip_map.left.insert(RobotIpMap::left_map::value_type(robot, ip));
}
