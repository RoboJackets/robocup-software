#include <rj_utils/logging.hpp>

#include "soccer_mom.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    rclcpp::spin(std::make_shared<tutorial::SoccerMom>());
}