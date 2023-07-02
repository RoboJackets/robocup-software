#include "manipulator_control.hpp"

#include <spdlog/spdlog.h>

#include "global_params.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

namespace control {

using soccer::robot::PARAM_max_chip_speed;
using soccer::robot::PARAM_max_kick_speed;
using soccer::robot::PARAM_min_safe_kick_power;

ManipulatorControl::ManipulatorControl(int shell_id, rclcpp::Node* node) : shell_id_(shell_id) {
    auto manipulator_pub = node->create_publisher<rj_msgs::msg::ManipulatorSetpoint>(
        topics::manipulator_setpoint_topic(shell_id), rclcpp::QoS(10));

    manipulator_pub_ = manipulator_pub;
    coach_state_sub_ = node->create_subscription<rj_msgs::msg::CoachState>(
        "/strategy/coach_state", rclcpp::QoS(1),
        [this](rj_msgs::msg::CoachState::SharedPtr coach_state) {  // NOLINT
            last_coach_state_ = *coach_state;
            dribbler_speed_ = last_coach_state_.global_override.max_dribbler_speed;
        });

    intent_sub_ = node->create_subscription<rj_msgs::msg::RobotIntent>(
        gameplay::topics::robot_intent_topic(shell_id), rclcpp::QoS(1),
        [this](rj_msgs::msg::RobotIntent::SharedPtr intent) {  // NOLINT
            manipulator_pub_->publish(rj_msgs::build<rj_msgs::msg::ManipulatorSetpoint>()
                                         .shoot_mode(intent->shoot_mode)
                                         .trigger_mode(intent->trigger_mode)
                                         .kick_speed(intent->kick_speed)
                                         .dribbler_speed(dribbler_speed_));
        });
}

}  // namespace control