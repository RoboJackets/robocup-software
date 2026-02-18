#include "rj_control/motion_control_node.hpp"

namespace control {

MotionControlNode::MotionControlNode()
    : rclcpp::Node("control", rclcpp::NodeOptions{}
                                  .automatically_declare_parameters_from_overrides(true)
                                  .allow_undeclared_parameters(true)),
      param_provider_(this, params::kMotionControlParamModule) {
    controllers_.reserve(kNumShells);

    auto drawing_publisher = create_publisher<rj_drawing_msgs::msg::DebugDraw>(
        viz::topics::kDebugDrawTopic, rclcpp::QoS(10));
    for (size_t i = 0; i < kNumShells; i++) {
        controllers_.emplace_back(static_cast<int>(i), this);
    }
}

}  // namespace control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto control = std::make_shared<control::MotionControlNode>();
    start_global_param_provider(control.get(), kGlobalParamServerNode);
    rclcpp::spin(control);
}
