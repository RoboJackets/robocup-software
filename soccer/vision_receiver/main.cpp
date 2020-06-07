#include <rclcpp/rclcpp.hpp>

#include "vision_receiver_sub.hpp"

using vision_receiver::VisionReceiver;

int main(int argc, char** argv) {
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VisionReceiver>());
    rclcpp::shutdown();
    return 0;
}
