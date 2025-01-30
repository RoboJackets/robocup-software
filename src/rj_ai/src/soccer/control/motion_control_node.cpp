#include "motion_control_node.hpp"

namespace control {

MotionControlNode::MotionControlNode()
    : rclcpp::Node("control", rclcpp::NodeOptions{}
                                  .automatically_declare_parameters_from_overrides(true)
                                  .allow_undeclared_parameters(true)),
      param_provider_(this, params::kMotionControlParamModule) {
    controllers_.reserve(kNumShells);

    auto drawing_publisher = create_publisher<rj_drawing_msgs::msg::DebugDraw>(
        viz::topics::kDebugDrawTopic, rclcpp::QoS(10));
    for (int i = 0; i < kNumShells; i++) {
        controllers_.emplace_back(i, this);
    }
}

}  // namespace control