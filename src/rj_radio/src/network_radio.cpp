#include "rj_radio/network_radio.hpp"

using namespace boost::asio;
using ip::udp;

namespace radio {

NetworkRadio::NetworkRadio()
    : control_message_socket_(io_service_),
      robot_status_socket_(io_service_),
      send_buffers_(kNumShells) {
    control_message_socket_.open(udp::v4());
    control_message_socket_.bind(udp::endpoint(udp::v4(), kControlMessageSocketPort));

    robot_status_socket_.open(udp::v4());
    robot_status_socket_.bind(udp::endpoint(udp::v4(), kRobotStatusMessageSocketPort));

    start_robot_status_receive();

    // Republish alive robots derived from RobotStatus reception (the same signal
    // the UI's robot list is built from). Replaces the old, buggy 8002 bitmask.
    alive_robots_timer_ = create_wall_timer(std::chrono::milliseconds(100),
                                            [this]() { publish_alive_from_status(); });
}

void NetworkRadio::start_robot_status_receive() {
    robot_status_socket_.async_receive_from(
        boost::asio::buffer(robot_status_buffer_), robot_status_endpoint_,
        [this](const boost::system::error_code& error, size_t num_bytes) {
            receive_robot_status(error, num_bytes);
        });
}

void NetworkRadio::send_control_message(uint8_t robot_id,
                                        const rj_msgs::msg::MotionSetpoint& motion,
                                        const rj_msgs::msg::ManipulatorSetpoint& manipulator,
                                        strategy::Positions role) {
    // Build the control packet for this robot.
    std::array<uint8_t, sizeof(RadioMessage::ControlMessage)>& forward_packet_buffer =
        send_buffers_[robot_id];

    // NOLINTNEXTLINE(cppcoreguidelines-pro-type-reinterpret-cast)
    auto* body = reinterpret_cast<RadioMessage::ControlMessage*>(&forward_packet_buffer[0]);

    ConvertTx::ros_to_rtp(manipulator, motion, robot_id, body, role, blue_team());

    control_message_socket_.async_send_to(
        boost::asio::buffer(forward_packet_buffer), control_message_endpoint_,
        [](const boost::system::error_code& error, [[maybe_unused]] std::size_t num_bytes) {
            if (static_cast<bool>(error)) {
                SPDLOG_ERROR("Error Sending: {}", error.message());
            }
        });
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
        start_robot_status_receive();
        return;
    }
    if (num_bytes != sizeof(RadioMessage::RobotStatusMessage)) {
        SPDLOG_ERROR("Invalid packet length: expected {}, got {}",
                     sizeof(RadioMessage::RobotStatusMessage), num_bytes);
        start_robot_status_receive();
        return;
    }

    auto* msg = reinterpret_cast<RadioMessage::RobotStatusMessage*>(&robot_status_buffer_[0]);

    int robot_id = msg->robot_id;

    // Extract the rtp to a regular struct.
    rj_msgs::msg::RobotStatus status_ros;
    RobotStatus status;
    ConvertRx::rtp_to_status(*msg, &status);
    ConvertRx::status_to_ros(status, &status_ros);

    publish_robot_status(robot_id, status_ros);

    // Restart receiving
    start_robot_status_receive();
}

void NetworkRadio::publish_alive_from_status() {
    const RJ::Time now = RJ::now();
    std::array<bool, kNumShells> alive_robots{};
    for (size_t robot_id = 0; robot_id < kNumShells; robot_id++) {
        alive_robots.at(robot_id) =
            last_status_received_.at(robot_id) + RJ::Seconds(PARAM_timeout) > now;
    }

    rj_msgs::msg::AliveRobots alive_message{};
    alive_message.alive_robots = alive_robots;
    publish_alive_robots(alive_message);
}

}  // namespace radio

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto radio = std::make_shared<radio::NetworkRadio>();
    start_global_param_provider(radio.get(), kGlobalParamServerNode);
    rclcpp::spin(radio);
}
