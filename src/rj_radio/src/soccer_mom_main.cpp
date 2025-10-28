#include "rclcpp/rclcpp.hpp"
#include "rj_radio/soccer_mom.hpp"

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<radio::SoccerMom>());
    rclcpp::shutdown();
    return 0;
}