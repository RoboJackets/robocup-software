#include "network_radio.hpp"

#include <boost/asio.hpp>
#include <fmt/ostream.h>
#include <spdlog/spdlog.h>

#include <rj_common/network.hpp>
#include <rj_common/status.hpp>
#include <rj_msgs/msg/alive_robots.hpp>

#include "packet_convert.hpp"
#include "rj_geometry/util.hpp"

using namespace boost::asio;
using ip::udp;

namespace radio {

NetworkRadio::NetworkRadio() : socket(io_service), recv_buffer_{}, send_buffers_(kNumShells), last_heard_from{} {
    socket.open(udp::v4());
    SPDLOG_INFO("Socket Opened");
    socket.bind(bound_endpoint);
    SPDLOG_INFO("Socket bound");

    start_receive();

    alive_robots_pub_ =
        this->create_publisher<rj_msgs::msg::AliveRobots>("strategy/alive_robots", rclcpp::QoS(1));
}

NetworkRadio::~NetworkRadio() {
    socket.close();
}

void NetworkRadio::start_receive() {
    socket.async_receive(
        boost::asio::buffer(recv_buffer_),
        [this](const boost::system::error_code& error, std::size_t num_bytes) {
            receive_packet(error, num_bytes);
        }
    );
}

void NetworkRadio::send(int robot_id, const rj_msgs::msg::MotionSetpoint& motion,
                        const rj_msgs::msg::ManipulatorSetpoint& manipulator,
                        strategy::Positions role) {
    // Build the control packet for this robot.
    std::array<uint8_t, rtp::HeaderSize + sizeof(rtp::RobotTxMessage)>& forward_packet_buffer =
        send_buffers_[robot_id];

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    auto* header = reinterpret_cast<rtp::Header*>(&forward_packet_buffer[0]);
    fill_header(header);

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    auto* body = reinterpret_cast<rtp::RobotTxMessage*>(&forward_packet_buffer[rtp::HeaderSize]);

    ConvertTx::ros_to_rtp(manipulator, motion, robot_id, body, role);

    socket.async_send_to(
        boost::asio::buffer(forward_packet_buffer), base_station_endpoint,
        [](const boost::system::error_code& error, [[maybe_unused]] std::size_t num_bytes) {
            if (static_cast<bool>(error)) {
                SPDLOG_ERROR("Error Sending: {}", error);
            }
        }
    );
}

void NetworkRadio::receive() {
    // Let boost::asio handle callbacks
    io_service.poll();
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

    int robot_id = msg->uid;

    last_heard_from[robot_id] = RJ::now();

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

void NetworkRadio::publish_alive_robots() {
    std::vector<u_int8_t> alive_robots = {};
    for (u_int8_t robot_id = 0; robot_id < kNumShells; robot_id++) {
        if (RJ::now() - last_heard_from[robot_id] < kTimeout) {
            alive_robots.push_back(robot_id);
        }
    }

    // publish a message containing the alive robots
    rj_msgs::msg::AliveRobots alive_message{};
    alive_message.alive_robots = alive_robots;
    alive_robots_pub_->publish(alive_message);
}

}  // namespace radio
