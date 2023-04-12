#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/manipulator_setpoint.hpp>
#include <rj_msgs/msg/robot_intent.hpp>
#include <rj_param_utils/param.hpp>
#include <rj_msgs/msg/coach_state.hpp>

namespace control {

class ManipulatorControl {
public:
    ManipulatorControl(int shell_id, rclcpp::Node* node);
    ~ManipulatorControl() = default;

    ManipulatorControl(ManipulatorControl&& other) = default;
    ManipulatorControl& operator=(ManipulatorControl&& other) = default;
    ManipulatorControl(const ManipulatorControl& other) = delete;
    ManipulatorControl& operator=(const ManipulatorControl& other) = delete;

private:
    int shell_id_;
    rclcpp::Publisher<rj_msgs::msg::ManipulatorSetpoint>::SharedPtr manipulator_pub_;
    rclcpp::Subscription<rj_msgs::msg::RobotIntent>::SharedPtr intent_sub_;
    rclcpp::Subscription<rj_msgs::msg::CoachState>::SharedPtr coach_state_sub_;

    rj_msgs::msg::CoachState last_coach_state_{};
};

}  // namespace control