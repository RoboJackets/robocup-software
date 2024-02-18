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

NetworkRadio::NetworkRadio()
    : base_station_socket_(io_service_),
      robot_status_buffer_{},
      alive_robots_buffer_{},
      send_buffers_(kNumShells) {
    base_station_socket_.open(udp::v4());
    base_station_socket_.bind(udp::endpoint(udp::v4(), kBaseStationBindPort));

    start_receive();
}

void NetworkRadio::start_receive() {
    base_station_socket_.async_receive_from(
        boost::asio::buffer(robot_status_buffer_), robot_status_endpoint_,
        [this](const boost::system::error_code& error, size_t num_bytes) {
            receive_robot_status(error, num_bytes);
        }
    );

    base_station_socket_.async_receive_from(
        boost::asio::buffer(alive_robots_buffer_), alive_robots_endpoint_,
        [this](const boost::system::error_code& error, size_t num_bytes) {
            receive_alive_robots(error, num_bytes);
        }
    );
}

void NetworkRadio::send_control_message(uint8_t robot_id, const rj_msgs::msg::MotionSetpoint& motion,
                        const rj_msgs::msg::ManipulatorSetpoint& manipulator,
                        strategy::Positions role) {
    // Build the control packet for this robot.
    std::array<uint8_t, sizeof(rtp::ControlMessage)>& forward_packet_buffer =
        send_buffers_[robot_id];

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    auto* body = reinterpret_cast<rtp::ControlMessage*>(&forward_packet_buffer[0]);

    ConvertTx::ros_to_rtp(manipulator, motion, robot_id, body, role, blue_team());

    base_station_socket_.async_send_to(
        boost::asio::buffer(forward_packet_buffer), control_message_endpoint_,
        [](const boost::system::error_code& error, [[maybe_unused]] std::size_t num_Bytes) {
            if (static_cast<bool>(error)) {
                SPDLOG_ERROR("Error Sending: {}", error.message());
            }
        }
    );
}

void NetworkRadio::poll_receive() {
    // Let boost::asio handle callbacks
    io_service_.poll();
}

void NetworkRadio::switch_team(bool blue_team) {
    // TODO (Nate): Send some command to the base station to switch teams.
}

void NetworkRadio::receive_robot_status(const boost::system::error_code& error, size_t num_bytes) {
    if (static_cast<bool>(error)) {
        SPDLOG_ERROR("Error Receiving Robot Status: {}.", error.message());
        return;
    }
    if (num_bytes != sizeof(rtp::RobotStatusMessage)) {
        SPDLOG_ERROR("Invalid packet length: expected {}, got {}", sizeof(rtp::RobotStatusMessage),
                     num_bytes);
        return;
    }

    auto* msg = reinterpret_cast<rtp::RobotStatusMessage*>(&robot_status_buffer_[0]);

    int robot_id = msg->robot_id;

    // Extract the rtp to a regular struct.
    rj_msgs::msg::RobotStatus status_ros;
    RobotStatus status;
    ConvertRx::rtp_to_status(*msg, &status);
    ConvertRx::status_to_ros(status, &status_ros);

    publish_robot_status(robot_id, status_ros);

    // Restart receiving
    start_receive();
}

void NetworkRadio::receive_alive_robots(const boost::system::error_code& error, size_t num_bytes) {
    if (static_cast<bool>(error)) {
        SPDLOG_ERROR("Error Receiving Alive Robots: {}", error.message());
        return;
    }

    if (num_bytes != 2) {
        SPDLOG_ERROR("Invalid Packet Length: expected {}, got {}", 2, num_bytes);
        return;
    }

    uint16_t alive = (alive_robots_buffer_[0] << 8) | (alive_robots_buffer_[0]);
    for (uint8_t robot_id = 0; robot_id < kNumShells; robot_id++) {
        if (alive & (1 << robot_id) != 0) {
            alive_robots_[robot_id] = true;
        } else {
            alive_robots_[robot_id] = false;
        }
    }

    rj_msgs::msg::AliveRobots alive_message{};
    alive_message.alive_robots = alive_robots_;
    publish_alive_robots(alive_message);
}

}  // namespace radio
