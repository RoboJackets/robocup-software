#include <rclcpp/rclcpp.hpp>

#include "SoccerMom.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto sm = std::make_shared<tutorial::SoccerMom>();
    rclcpp::spin(sm);
}
