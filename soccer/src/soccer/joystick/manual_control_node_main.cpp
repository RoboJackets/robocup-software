#include <rclcpp/rclcpp.hpp>

#include <rj_utils/logging.hpp>

#include "manual_control_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto manual = std::make_shared<joystick::ManualControlNode>();
    rclcpp::spin(manual);
}
