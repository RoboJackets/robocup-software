#include <rj_utils/logging.hpp>

#include "coach_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");
    auto options = rclcpp::NodeOptions{};
    auto coach = std::make_shared<CoachNode>(options);
    rclcpp::spin(coach);
}