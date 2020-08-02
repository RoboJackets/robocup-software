#include <rclcpp/rclcpp.hpp>

#include "InternalReferee.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto ref = std::make_shared<referee::InternalReferee>();
    rclcpp::spin(ref);
}
