#include "network_radio.hpp"

#include <boost/asio.hpp>
#include <fmt/ostream.h>
#include <spdlog/spdlog.h>

#include <rj_common/network.hpp>
#include <rj_common/status.hpp>

#include "packet_convert.hpp"
#include "rj_geometry/util.hpp"

using namespace boost::asio;
using ip::udp;

namespace radio {

NetworkRadio::NetworkRadio() : socket_(io_service_), recv_buffer_{}, send_buffers_(kNumShells) {
    connections_.resize(kNumShells);

    this->get_parameter("server_port", param_server_port_);
    SPDLOG_INFO("Radio param_server_port_: {}", param_server_port_);

    // socket must be opened before it can be bound to an endpoint
    socket_.open(udp::v4());
    socket_.bind(udp::endpoint(udp::v4(), param_server_port_));

    start_receive();
}

void NetworkRadio::start_receive() {
    // Set a receive callback
    socket_.async_receive_from(boost::asio::buffer(recv_buffer_), robot_endpoint_,
                               [this](const boost::system::error_code& error,
                                      std::size_t num_bytes) { receive_packet(error, num_bytes); });
}

void NetworkRadio::send(int robot_id, const rj_msgs::msg::MotionSetpoint& motion,
                        const rj_msgs::msg::ManipulatorSetpoint& manipulator) {
    // Build the control packet for this robot.
    std::array<uint8_t, rtp::HeaderSize + sizeof(rtp::RobotTxMessage)>& forward_packet_buffer =
        send_buffers_[robot_id];

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    auto* header = reinterpret_cast<rtp::Header*>(&forward_packet_buffer[0]);
    fill_header(header);

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    auto* body = reinterpret_cast<rtp::RobotTxMessage*>(&forward_packet_buffer[rtp::HeaderSize]);

    ConvertTx::ros_to_rtp(manipulator, motion, robot_id, body);

    // Fetch the connection
    auto maybe_connection = connections_.at(robot_id);

    // If there exists a connection, we can send.
    if (maybe_connection) {
        const RobotConnection& connection = maybe_connection.value();
        // Check if we've timed out.
        if (RJ::now() + kTimeout < connection.last_received) {
            // Remove the endpoint from the IP map and the connection list
            assert(robot_ip_map_.erase(connection.endpoint) == 1);  // NOLINT
            connections_.at(robot_id) = std::nullopt;
        } else {
            // Send to the given IP address
            const udp::endpoint& robot_endpoint = connection.endpoint;
            socket_.async_send_to(
                boost::asio::buffer(forward_packet_buffer), robot_endpoint,
                [](const boost::system::error_code& error, [[maybe_unused]] std::size_t num_bytes) {
                    // Handle errors.
                    if (static_cast<bool>(error)) {
                        SPDLOG_ERROR(  // NOLINT(bugprone-lambda-function-name)
                            "Error sending: {}.", error);
                    }
                });
        }
    }
}

void NetworkRadio::receive() {
    // Let boost::asio handle callbacks
    io_service_.poll();
}

void NetworkRadio::receive_packet(const boost::system::error_code& error, std::size_t num_bytes) {
    if (static_cast<bool>(error)) {
        SPDLOG_ERROR("Error sending: {}.", error);
        return;
    }
    if (num_bytes != rtp::ReverseSize) {
        SPDLOG_ERROR("Invalid packet length: expected {}, got {}", rtp::ReverseSize, num_bytes);
        return;
    }

    auto* msg = reinterpret_cast<rtp::RobotStatusMessage*>(&recv_buffer_[rtp::HeaderSize]);

    robot_endpoint_.port(kRobotEndpointPort);

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
    rj_msgs::msg::RobotStatus status_ros;
    RobotStatus status;
    ConvertRx::rtp_to_status(*msg, &status);
    ConvertRx::status_to_ros(status, &status_ros);

    publish(robot_id, status_ros);

    // Restart receiving
    start_receive();
}

void NetworkRadio::switch_team(bool /*blue_team*/) {}

}  // namespace radio
