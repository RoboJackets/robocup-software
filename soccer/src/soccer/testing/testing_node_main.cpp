#include <rj_utils/logging.hpp>

#include "testing_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");
    auto options = rclcpp::NodeOptions{};
    auto testing_node = std::make_shared<TestingNode>(options);
    rclcpp::spin(testing_node);
}
