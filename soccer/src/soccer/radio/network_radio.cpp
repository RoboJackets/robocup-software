#include "network_radio.hpp"

#include <rj_common/status.hpp>

#include "Geometry2d/Util.hpp"
#include "packet_convert.hpp"

using namespace boost::asio;
using ip::udp;

NetworkRadio::NetworkRadio(int server_port)
    : socket_(context_, udp::endpoint(udp::v4(), server_port)),
      recv_buffer_{},
      send_buffers_(kNumShells) {
    connections_.resize(kNumShells);
    start_receive();
}

void NetworkRadio::start_receive() {
    // Set a receive callback
    socket_.async_receive_from(boost::asio::buffer(recv_buffer_), robot_endpoint_,
                               [this](const boost::system::error_code& error,
                                      std::size_t num_bytes) { receive_packet(error, num_bytes); });
}

bool NetworkRadio::is_open() const { return socket_.is_open(); }

void NetworkRadio::send(const std::array<RobotIntent, kNumShells>& intents,
                        const std::array<MotionSetpoint, kNumShells>& setpoints) {
    for (int shell = 0; shell < kNumShells; shell++) {
        const auto& intent = intents[shell];
        if (!intent.is_active) {
            continue;
        }

        const auto& setpoint = setpoints[shell];

        // Build the control packet for this robot.
        std::array<uint8_t, rtp::HeaderSize + sizeof(rtp::RobotTxMessage)>& forward_packet_buffer =
            send_buffers_[shell];

        auto* header = reinterpret_cast<rtp::Header*>(&forward_packet_buffer[0]);
        fill_header(header);

        auto* body =
            reinterpret_cast<rtp::RobotTxMessage*>(&forward_packet_buffer[rtp::HeaderSize]);

        ConvertTx::to_rtp(intent, setpoint, shell, body);

        // Fetch the connection
        auto maybe_connection = connections_.at(shell);

        // If there exists a connection, we can send.
        if (maybe_connection) {
            const RobotConnection& connection = maybe_connection.value();
            // Check if we've timed out.
            if (RJ::now() + kTimeout < connection.last_received) {
                // Remove the endpoint from the IP map and the connection list
                assert(robot_ip_map_.erase(connection.endpoint) == 1);
                connections_.at(shell) = std::nullopt;
            } else {
                // Send to the given IP address
                const udp::endpoint& robot_endpoint = connection.endpoint;
                socket_.async_send_to(
                    boost::asio::buffer(forward_packet_buffer), robot_endpoint,
                    [](const boost::system::error_code& error, std::size_t num_bytes) {
                        // Handle errors.
                        if (static_cast<bool>(error)) {
                            std::cerr << "Error sending: " << error << " in " __FILE__ << std::endl;
                        }
                    });
            }
        }
    }
}

void NetworkRadio::receive() {
    // Let boost::asio handle callbacks
    context_.poll();
}

void NetworkRadio::receive_packet(const boost::system::error_code& error, std::size_t num_bytes) {
    if (static_cast<bool>(error)) {
        std::cerr << "Error receiving: " << error << " in " __FILE__ << std::endl;
        return;
    }
    if (num_bytes != rtp::ReverseSize) {
        std::cerr << "Invalid packet length: expected " << rtp::ReverseSize << ", got " << num_bytes
                  << std::endl;
        return;
    }

    auto* msg = reinterpret_cast<rtp::RobotStatusMessage*>(&recv_buffer_[rtp::HeaderSize]);

    robot_endpoint_.port(25566);

    int robot_id = msg->uid;

    auto iter = robot_ip_map_.find(robot_endpoint_);
    if (iter != robot_ip_map_.end() && iter->second != robot_id) {
        // Make sure this IP address isn't mapped to another robot ID.
        // If it is, remove the entry and the connections corresponding
        // to both this ID and this IP address.
        connections_.at(iter->second) = std::nullopt;
        robot_ip_map_.erase(iter);
        connections_.at(robot_id) = std::nullopt;
    }

    // Update assignments.
    if (!connections_.at(robot_id)) {
        connections_.at(robot_id) = RobotConnection{robot_endpoint_, RJ::now()};
        robot_ip_map_.insert({robot_endpoint_, robot_id});
    } else {
        // Update the timeout watchdog
        connections_.at(robot_id)->last_received = RJ::now();
    }

    // Extract the rtp to a regular struct.
    RobotStatus status;
    ConvertRx::rtp_to_status(*msg, &status);

    {
        // Add reverse packets
        std::lock_guard<std::mutex> lock(reverse_packets_mutex_);
        reverse_packets_.push_back(status);
    }

    // Restart receiving
    start_receive();
}

void NetworkRadio::switch_team(bool /*blue_team*/) {}
