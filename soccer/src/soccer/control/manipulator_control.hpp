#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/manipulator_setpoint.hpp>
#include <rj_msgs/msg/robot_intent.hpp>
#include <rj_param_utils/param.hpp>

namespace control {

DECLARE_FLOAT64(params::kMotionControlParamModule, max_kick_speed);
DECLARE_INT64(params::kMotionControlParamModule, min_safe_kick_power);

class ManipulatorControl {
public:
    ManipulatorControl(RobotId shell_id, rclcpp::Node* node);
    ~ManipulatorControl() = default;

    ManipulatorControl(ManipulatorControl&& other) = default;
    ManipulatorControl& operator=(ManipulatorControl&& other) = default;
    ManipulatorControl(const ManipulatorControl& other) = delete;
    ManipulatorControl& operator=(const ManipulatorControl& other) = delete;

private:
    RobotId shell_id_;
    rclcpp::Publisher<rj_msgs::msg::ManipulatorSetpoint>::SharedPtr manipulator_pub_;
    rclcpp::Subscription<rj_msgs::msg::RobotIntent>::SharedPtr intent_sub_;
};

}  // namespace control