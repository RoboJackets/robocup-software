#pragma once

#include <array>
#include <chrono>
#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/manipulator_setpoint.hpp>
#include <rj_msgs/msg/motion_setpoint.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "rj_joystick/manual_control.hpp"

namespace joystick {

/**
 * Keeps track of joysticks (and other manual control methods) and assigning them to robots.
 * Publishes on manipulator and motion setpoints, only for the robots being controlled.
 */
class ManualControlNode : public rclcpp::Node {
public:
    ManualControlNode();

private:
    void tick();
    void stop_robot(int robot_id);
    void publish(int robot_id, const ControllerCommand& command);

    std::shared_ptr<ManualControlParams> params_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    std::vector<rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr> joy_subs_;
    std::vector<std::chrono::time_point<std::chrono::steady_clock>> last_command_times_;
    std::vector<double> kick_power_;
    std::vector<double> dribble_power_;
    std::vector<ControllerCommand> commands_;
    std::vector<std::array<bool, 4>> previous_power_buttons_;
    std::vector<bool> has_command_;
    rclcpp::TimerBase::SharedPtr tick_timer_;

    std::vector<rclcpp::Publisher<rj_msgs::msg::MotionSetpoint>::SharedPtr> motion_setpoint_pubs_;
    std::vector<rclcpp::Publisher<rj_msgs::msg::ManipulatorSetpoint>::SharedPtr>
        manipulator_setpoint_pubs_;
};

}  // namespace joystick