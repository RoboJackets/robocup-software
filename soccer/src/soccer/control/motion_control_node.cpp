#include "motion_control_node.hpp"

#include <spdlog/spdlog.h>

#include "robot.hpp"

namespace control {

MotionControlNode::MotionControlNode()
    : rclcpp::Node("control"), param_provider_(this, params::kMotionControlParamModule) {
    controllers_.reserve(kNumShells);

    auto drawing_publisher = create_publisher<rj_drawing_msgs::msg::DebugDraw>(
        viz::topics::kDebugDrawPub, rclcpp::QoS(10));
    for (RobotId i = 0; i < kNumShells; i++) {
        controllers_.emplace_back(i, this);
        manipulators_.emplace_back(i, this);
    }
}

}  // namespace control