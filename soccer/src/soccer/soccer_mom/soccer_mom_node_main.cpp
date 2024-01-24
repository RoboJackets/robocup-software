#include "soccer_mom.hpp"
#include <rclcpp/rclcpp.hpp>
#include "std_msgs/msg/string.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto soccer_mom = std::make_shared<tutorial::SoccerMom>();
    rclcpp::spin(soccer_mom);
}