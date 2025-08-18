#include "rj_referee/external_referee.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto ref = std::make_shared<referee::ExternalReferee>();
    rclcpp::spin(ref);
    rclcpp::shutdown();
    return 0;
}