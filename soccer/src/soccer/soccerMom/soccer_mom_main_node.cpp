#include <rclcpp/rclcpp.hpp>
#include "soccerMom.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto mom = std::make_shared<tutorial::SoccerMom>();
    rclcpp::spin(mom);
}