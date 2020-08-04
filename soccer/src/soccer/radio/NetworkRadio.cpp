#include "NetworkRadio.hpp"

#include <rj_common/status.h>

#include "Geometry2d/Util.hpp"
#include "PacketConvert.hpp"

using namespace boost::asio;
using ip::udp;

NetworkRadio::NetworkRadio(int server_port)
    : _socket(_context, udp::endpoint(udp::v4(), server_port)),
      _recv_buffer{},
      _send_buffers(Num_Shells) {
    _connections.resize(Num_Shells);
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

void NetworkRadio::send(
    const std::array<RobotIntent, Num_Shells>& intents,
    const std::array<MotionSetpoint, Num_Shells>& setpoints) {
    for (int shell = 0; shell < Num_Shells; shell++) {
        const auto& intent = intents[shell];
        if (!intent.is_active) {
            continue;
        }

        const auto& setpoint = setpoints[shell];

        // Build the control packet for this robot.
        std::array<uint8_t, rtp::HeaderSize + sizeof(rtp::RobotTxMessage)>&
            forward_packet_buffer = _send_buffers[shell];

        auto* header =
            reinterpret_cast<rtp::Header*>(&forward_packet_buffer[0]);
        fill_header(header);

        auto* body = reinterpret_cast<rtp::RobotTxMessage*>(
            &forward_packet_buffer[rtp::HeaderSize]);

        ConvertTx::to_rtp(intent, setpoint, shell, body);

        // Fetch the connection
        auto maybe_connection = _connections.at(shell);

        // If there exists a connection, we can send.
        if (maybe_connection) {
            const RobotConnection& connection = maybe_connection.value();
            // Check if we've timed out.
            if (RJ::now() + kTimeout < connection.last_received) {
                // Remove the endpoint from the IP map and the connection list
                assert(_robot_ip_map.erase(connection.endpoint) == 1);
                _connections.at(shell) = std::nullopt;
            } else {
                // Send to the given IP address
                const udp::endpoint& robot_endpoint = connection.endpoint;
                _socket.async_send_to(
                    boost::asio::buffer(forward_packet_buffer), robot_endpoint,
                    [](const boost::system::error_code& error,
                       std::size_t num_bytes) {
                        // Handle errors.
                        if (static_cast<bool>(error)) {
                            std::cerr << "Error sending: " << error
                                      << " in " __FILE__ << std::endl;
                        }
                    });
            }
        }
    }
}

void NetworkRadio::receive() {
    // Let boost::asio handle callbacks
    _context.poll();
}

void NetworkRadio::receivePacket(const boost::system::error_code& error,
                                 std::size_t num_bytes) {
    if (static_cast<bool>(error)) {
        std::cerr << "Error receiving: " << error << " in " __FILE__
                  << std::endl;
        return;
    }
    if (num_bytes != rtp::ReverseSize) {
        std::cerr << "Invalid packet length: expected " << rtp::ReverseSize
                  << ", got " << num_bytes << std::endl;
        return;
    }

    auto* msg = reinterpret_cast<rtp::RobotStatusMessage*>(
        &_recv_buffer[rtp::HeaderSize]);

    _robot_endpoint.port(25566);

    int robot_id = msg->uid;

    auto iter = _robot_ip_map.find(_robot_endpoint);
    if (iter != _robot_ip_map.end() && iter->second != robot_id) {
        // Make sure this IP address isn't mapped to another robot ID.
        // If it is, remove the entry and the connections corresponding
        // to both this ID and this IP address.
        _connections.at(iter->second) = std::nullopt;
        _robot_ip_map.erase(iter);
        _connections.at(robot_id) = std::nullopt;
    }

    // Update assignments.
    if (!_connections.at(robot_id)) {
        _connections.at(robot_id) = RobotConnection{_robot_endpoint, RJ::now()};
        _robot_ip_map.insert({_robot_endpoint, robot_id});
    } else {
        // Update the timeout watchdog
        _connections.at(robot_id)->last_received = RJ::now();
    }

    // Extract the rtp to a regular struct.
    RobotStatus status;
    ConvertRx::rtp_to_status(*msg, &status);

    {
        // Add reverse packets
        std::lock_guard<std::mutex> lock(_reverse_packets_mutex);
        _reversePackets.push_back(status);
    }

    // Restart receiving
    startReceive();
}

void NetworkRadio::switchTeam(bool /*blueTeam*/) {}
