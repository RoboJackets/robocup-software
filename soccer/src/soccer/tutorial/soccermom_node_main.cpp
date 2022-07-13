#include <rclcpp/rclcpp.hpp>

#include <rj_utils/logging.hpp>

#include "soccermom_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto soccermom = std::make_shared<tutorial::SoccerMomNode>();
    rclcpp::spin(soccermom);
}
