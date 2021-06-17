#include <rclcpp/rclcpp.hpp>

#include "external_referee.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto ref = std::make_shared<referee::ExternalReferee>();
    rclcpp::spin(ref);
}