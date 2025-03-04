#include <rclcpp/rclcpp.hpp>
#include "kicker_picker.hpp"

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<strategy::KickerPicker>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
