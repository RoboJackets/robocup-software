#include "manipulator_control.hpp"

#include <spdlog/spdlog.h>

#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

namespace control {

DEFINE_FLOAT64(params::kMotionControlParamModule, max_kick_speed, 5.0,
               "Maximum speed of the kicker (in m/s)");
DEFINE_INT64(params::kMotionControlParamModule, min_safe_kick_power, 64,
             "Minimum safe discharge power for the kicker (0-255)");

ManipulatorControl::ManipulatorControl(int shell_id, rclcpp::Node* node) : shell_id_(shell_id) {
    auto manipulator_pub = node->create_publisher<rj_msgs::msg::ManipulatorSetpoint>(
        topics::manipulator_setpoint_pub(shell_id), rclcpp::QoS(10));

    manipulator_pub_ = manipulator_pub;
    intent_sub_ = node->create_subscription<rj_msgs::msg::RobotIntent>(
        gameplay::topics::robot_intent_pub(shell_id), rclcpp::QoS(1),
        [manipulator_pub](rj_msgs::msg::RobotIntent::SharedPtr intent) {  // NOLINT
            int kick_strength =
                std::max<int>(PARAM_min_safe_kick_power,
                              std::min<int>(255, intent->kick_speed / PARAM_max_kick_speed));
            manipulator_pub->publish(rj_msgs::build<rj_msgs::msg::ManipulatorSetpoint>()
                                         .shoot_mode(intent->shoot_mode)
                                         .trigger_mode(intent->trigger_mode)
                                         .kick_strength(kick_strength)
                                         .dribbler_speed(intent->dribbler_speed));
        });
}

}  // namespace control