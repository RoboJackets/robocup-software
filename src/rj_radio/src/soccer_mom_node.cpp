#include "rj_radio/soccer_mom.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<tutorial::SoccerMom>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}