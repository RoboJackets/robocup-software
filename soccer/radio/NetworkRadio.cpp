#include "NetworkRadio.hpp"
#include <Utils.hpp>
#include "Geometry2d/Util.hpp"
#include "PacketConvert.hpp"
#include "status.h"

using namespace boost::asio;
using ip::udp;

NetworkRadio::NetworkRadio(int server_port)
    : _socket(_context, udp::endpoint(udp::v4(), server_port)),
      _send_buffers(Robots_Per_Team) {
    startReceive();
}

void NetworkRadio::startReceive() {
    // Set a receive callback
    _socket.async_receive_from(
        boost::asio::buffer(_recv_buffer), _robot_endpoint,
        [this](const boost::system::error_code& error, std::size_t num_bytes) {
            receivePacket(error, num_bytes);
        });
}

bool NetworkRadio::isOpen() const { return _socket.is_open(); }

void NetworkRadio::send(Packet::RadioTx& packet) {
    // Get a list of all the IP addresses this packet needs to be sent to
    for (int robot_idx = 0; robot_idx < packet.robots_size(); robot_idx++) {
        const Packet::Control& control = packet.robots(robot_idx).control();
        uint32_t robot_id = packet.robots(robot_idx).uid();

        // Build the control packet for this robot.
        std::array<uint8_t, rtp::HeaderSize + sizeof(rtp::RobotTxMessage)>&
            forward_packet_buffer = _send_buffers[robot_idx];

        rtp::Header* header =
            reinterpret_cast<rtp::Header*>(&forward_packet_buffer[0]);
        fill_header(header);

        rtp::RobotTxMessage* body = reinterpret_cast<rtp::RobotTxMessage*>(
            &forward_packet_buffer[rtp::HeaderSize]);

        from_robot_tx_proto(packet.robots(robot_idx), body);

        // Fetch the IP address
        auto range = _robot_ip_map.left.equal_range(robot_id);

        // Loop over all addresses for this robot; we want to send multiple
        // copies.
        for (auto it = range.first; it != range.second; it++) {
            // Send to the given IP address
            const udp::endpoint& robot_endpoint = it->second;
            _socket.async_send_to(
                boost::asio::buffer(forward_packet_buffer), robot_endpoint,
                [](const boost::system::error_code& error,
                   std::size_t num_bytes) {
                    // Handle errors.
                    if (error) {
                        std::cerr << "Error sending: " << error
                                  << " in " __FILE__ << std::endl;
                    }
                });
        }
    }
}

void NetworkRadio::receive() {
    // Let boost::asio handles callbacks
    _context.poll();
}

void NetworkRadio::receivePacket(const boost::system::error_code& error,
                                 std::size_t num_bytes) {
    if (error) {
        std::cerr << "Error receiving: " << error << " in " __FILE__
                  << std::endl;
        return;
    } else if (num_bytes != rtp::ReverseSize) {
        std::cerr << "Invalid packet length: expected " << rtp::ReverseSize
                  << ", got " << num_bytes << std::endl;
        return;
    }

    rtp::RobotStatusMessage* msg = reinterpret_cast<rtp::RobotStatusMessage*>(
        &_recv_buffer[rtp::HeaderSize]);

    _robot_endpoint.port(25566);

    // Find out which robot this corresponds to.
    auto ip_iter = _robot_ip_map.right.find(_robot_endpoint);

    int robot_id = msg->uid;

    // We already have this robot registered; this is a reverse packet
    if (ip_iter == _robot_ip_map.right.end()) {
        // This is a new robot, so we need to register this robot's UID
        std::cout << "Adding robot with endpoint " << _robot_endpoint
                  << std::endl;
        registerRobot(msg->uid, _robot_endpoint);
    } else if (ip_iter->second != robot_id) {
        // This robot has been reassigned. Re-registering it sets the IP->ID
        // map correctly, but we still need to remove it from the ID->IP map.
        std::cerr << "Warning: UID of robot assigned IP " << ip_iter->first
                  << " changed to " << ip_iter->second << ". Reassigning."
                  << std::endl;
        ;

        // Erase the IP address and re-register the robot.
        _robot_ip_map.right.erase(ip_iter->first);
        registerRobot(msg->uid, _robot_endpoint);
    }

    // Extract the protobuf form
    Packet::RadioRx packet = convert_rx_rtp_to_proto(*msg);

    {
        // Add reverse packets
        std::lock_guard<std::mutex> lock(_reverse_packets_mutex);
        _reversePackets.push_back(packet);
    }

    // Restart receiving
    startReceive();
}

void NetworkRadio::switchTeam(bool) {}

void NetworkRadio::registerRobot(int robot, udp::endpoint endpoint) {
    // Insert the robot into the bimap, going both directions
    _robot_ip_map.insert({robot, endpoint});
}
