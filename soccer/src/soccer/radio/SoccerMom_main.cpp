#include <rclcpp/rclcpp.hpp>

#include "SoccerMom.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto SoccerMom = std::make_shared<tutorial::SoccerMom>();
    rclcpp::spin(SoccerMom);
}