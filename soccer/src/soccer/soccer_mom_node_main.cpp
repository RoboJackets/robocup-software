#include "soccer_mom.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto soccer_mom = std::make_shared<soccer_mom::SoccerMom>();
    rclcpp::spin(soccer_mom);
}
