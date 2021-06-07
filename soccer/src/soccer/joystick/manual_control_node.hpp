#pragma once

#include <functional>
#include <unordered_map>

#include <SDL.h>
#include <fmt/format.h>
#include <spdlog/spdlog.h>

#include <node.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/manipulator_setpoint.hpp>
#include <rj_msgs/msg/motion_setpoint.hpp>
#include <rj_msgs/srv/list_joysticks.hpp>
#include <rj_msgs/srv/set_manual.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>

#include "context.hpp"
#include "manual_control.hpp"
#include "sdl_manual_control.hpp"

namespace joystick {

/**
 * Keeps track of joysticks (and other manual control methods) and assigning them to robots.
 *
 * Services:
 *  - ListJoysticks: get a list of joysticks and robots to which they are assigned
 *  - SetManual: assign a particular robot to a joystick and remove the robot previously assigned to
 * that joystick, if any
 *
 * Publishes on manipulator and motion setpoints, only for the robots being controlled.
 */
class ManualControlNode : public rclcpp::Node {
public:
    ManualControlNode();

private:
    void stop_robot(int robot_id);
    void publish(int robot_id, const ControllerCommand& command);

    void set_manual(const std::string& uuid, std::optional<int> robot_id);
    void remove_controller(ManualController* controller);

    std::vector<std::unique_ptr<ManualControllerProvider>> providers_;
    std::map<ManualController*, std::optional<int>> controllers_;

    rclcpp::Service<rj_msgs::srv::ListJoysticks>::SharedPtr list_joysticks_;
    rclcpp::Service<rj_msgs::srv::SetManual>::SharedPtr set_manual_;

    std::vector<rclcpp::Publisher<rj_msgs::msg::MotionSetpoint>::SharedPtr> motion_setpoint_pubs_;
    std::vector<rclcpp::Publisher<rj_msgs::msg::ManipulatorSetpoint>::SharedPtr>
        manipulator_setpoint_pubs_;

    rclcpp::TimerBase::SharedPtr timer_;

    ::params::LocalROS2ParamProvider param_provider_;
};

}  // namespace joystick
