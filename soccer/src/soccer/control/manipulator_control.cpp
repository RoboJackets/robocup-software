#include "manipulator_control.hpp"

#include <spdlog/spdlog.h>

#include "global_params.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"

namespace control {

using soccer::robot::PARAM_max_chip_speed;
using soccer::robot::PARAM_max_kick_speed;
using soccer::robot::PARAM_min_chip_speed;
using soccer::robot::PARAM_min_kick_speed;
using soccer::robot::PARAM_min_safe_kick_power;

ManipulatorControl::ManipulatorControl(int shell_id, rclcpp::Node* node) : shell_id_(shell_id) {
    auto manipulator_pub = node->create_publisher<rj_msgs::msg::ManipulatorSetpoint>(
        topics::manipulator_setpoint_pub(shell_id), rclcpp::QoS(10));

    manipulator_pub_ = manipulator_pub;
    intent_sub_ = node->create_subscription<rj_msgs::msg::RobotIntent>(
        gameplay::topics::robot_intent_pub(shell_id), rclcpp::QoS(1),
        [manipulator_pub](rj_msgs::msg::RobotIntent::SharedPtr intent) {  // NOLINT
            const double max_speed =
                intent->shoot_mode == rj_msgs::msg::RobotIntent::SHOOT_MODE_KICK
                    ? PARAM_max_kick_speed
                    : PARAM_max_chip_speed;
            int kick_strength =
                std::max<int>(PARAM_min_safe_kick_power,
                              std::min<int>(255, intent->kick_speed / max_speed * 255));
            manipulator_pub->publish(rj_msgs::build<rj_msgs::msg::ManipulatorSetpoint>()
                                         .shoot_mode(intent->shoot_mode)
                                         .trigger_mode(intent->trigger_mode)
                                         .kick_strength(kick_strength)
                                         .dribbler_speed(intent->dribbler_speed));
        });
}

}  // namespace control