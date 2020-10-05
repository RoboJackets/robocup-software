#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/manipulator_setpoint.hpp>
#include <rj_msgs/msg/robot_intent.hpp>
#include <rj_param_utils/param.hpp>

#include <rj_constants/topic_names.hpp>

namespace control {

DECLARE_FLOAT64(params::kMotionControlParamModule, max_kick_speed);
DECLARE_INT64(params::kMotionControlParamModule, min_safe_kick_power);

class ManipulatorControl {
public:
    ManipulatorControl(int shell_id, rclcpp::Node::SharedPtr node);

private:
    int shell_id_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Publisher<rj_msgs::msg::ManipulatorSetpoint>::SharedPtr manipulator_pub_;
    rclcpp::Subscription<rj_msgs::msg::RobotIntent>::SharedPtr intent_sub_;
};

} // namespace control