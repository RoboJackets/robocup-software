#include <rclcpp/rclcpp.hpp>

#include "marking.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<strategy::Marking>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
