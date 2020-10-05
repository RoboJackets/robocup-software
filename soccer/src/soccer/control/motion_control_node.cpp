#include "motion_control_node.hpp"

#include "robot.hpp"

namespace control {

MotionControlNode::MotionControlNode()
    : rclcpp::Node("control"),
      param_provider_(this, params::kMotionControlParamModule) {
    controllers_.reserve(kNumShells);
    for (int i = 0; i < kNumShells; i++) {
        controllers_.emplace_back(i, create_sub_node("motion_control_" + std::to_string(i)), nullptr);
        manipulators_.emplace_back(i, create_sub_node("manipulator_control_" + std::to_string(i)));
    }
}

} // namespace control