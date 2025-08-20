#include <rclcpp/rclcpp.hpp>

#include "rj_vision_receiver/vision_receiver.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<vision_receiver::VisionReceiver>());
    rclcpp::shutdown();
    return 0;
}