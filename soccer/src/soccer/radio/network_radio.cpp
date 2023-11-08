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
    socket.bind(udp::endpoint(udp::v4(), kIncomingBaseStationDataPort));

    start_receive();

    alive_robots_pub_ =
        this->create_publisher<rj_msgs::msg::AliveRobots>("strategy/alive_robots", rclcpp::QoS(1));
}

void NetworkRadio::start_receive() {
    socket.async_receive_from(
        boost::asio::buffer(recv_buffer_), bound_endpoint,
        [this](const boost::system::error_code& error, std::size_t num_bytes) {
            receive_packet(error, num_bytes);
        }
    );
}

void NetworkRadio::send(int robot_id, const rj_msgs::msg::MotionSetpoint& motion,
                        const rj_msgs::msg::ManipulatorSetpoint& manipulator,
                        strategy::Positions role) {

    // Build the control packet for this robot.
    std::array<uint8_t, sizeof(rtp::ControlMessage)>& forward_packet_buffer =
        send_buffers_[robot_id];

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    auto* body = reinterpret_cast<rtp::ControlMessage*>(&forward_packet_buffer[0]);

    ConvertTx::ros_to_rtp(manipulator, motion, robot_id, body, role, _blue_team);

    if (robot_id == 1) {
        SPDLOG_INFO("Team: {}, Robot id: {}, shoot_mode: {}, trigger_mode: {}, body_x: {}, body_y: {}, body_w: {}, dribbler_speed: {}, kick_strength: {}, role: {}, unused: {}",
                        body->team, body->robot_id, body->shoot_mode, body->trigger_mode, body->body_x, body->body_y, body->body_w, body->dribbler_speed, body->kick_strength, body->role, body->unused);
        SPDLOG_INFO("[{}, {}, {}, {}, {}, {}, {}, {}, {}, {}]", forward_packet_buffer[0], forward_packet_buffer[1], forward_packet_buffer[2], forward_packet_buffer[3], forward_packet_buffer[4], forward_packet_buffer[5], forward_packet_buffer[6], forward_packet_buffer[7], forward_packet_buffer[8], forward_packet_buffer[9]);
    }

    socket.async_send_to(
        boost::asio::buffer(forward_packet_buffer), base_station_endpoint,
        [](const boost::system::error_code& error, [[maybe_unused]] std::size_t num_bytes) {
            if (static_cast<bool>(error)) {
                SPDLOG_ERROR("Error Sending: {}", error.message());
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
    if (num_bytes != sizeof(rtp::RobotStatusMessage)) {
        SPDLOG_ERROR("Invalid packet length: expected {}, got {}", sizeof(rtp::RobotStatusMessage), num_bytes);
        return;
    }

    auto* msg = reinterpret_cast<rtp::RobotStatusMessage*>(&recv_buffer_[0]);

    int robot_id = msg->robot_id;

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

void NetworkRadio::switch_team(bool blue_team) {
    _blue_team = blue_team;
}

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
