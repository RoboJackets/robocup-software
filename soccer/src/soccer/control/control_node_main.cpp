#include "motion_control_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto control = std::make_shared<control::MotionControlNode>();
    rclcpp::spin(control);
}
