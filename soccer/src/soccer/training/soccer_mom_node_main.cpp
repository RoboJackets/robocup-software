#include <rj_utils/logging.hpp>

#include "soccer_mom_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto soccerMom = std::make_shared<SoccerMom>();
    rclcpp::spin(soccerMom);
}