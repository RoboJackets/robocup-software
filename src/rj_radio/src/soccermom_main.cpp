#include "rj_radio/soccermom.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto tutorial = std::make_shared<tutorial::SoccerMom>();
    rclcpp::spin(tutorial);
}