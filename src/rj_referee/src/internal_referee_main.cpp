#include <rclcpp/rclcpp.hpp>

#include "rj_referee/internal_referee.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto ref = std::make_shared<referee::InternalReferee>();
    rclcpp::spin(ref);
    rclcpp::shutdown();
    return 0;
}