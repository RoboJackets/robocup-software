#pragma once

#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <spdlog/spdlog.h>

#include <rj_common/ros_debug_drawer.hpp>
#include <rj_common/game_state.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/controller_select.hpp>
#include <rj_msgs/msg/goalie.hpp>
#include <rj_utils/logging.hpp>
#include <rj_utils/parsing.hpp>
#include <rj_control_extensions/action.hpp>
#include <rj_control_extensions/control_command.hpp>

#include "rj_control/controllers/controller.hpp"
#include "rj_control/controllers/position_controller.hpp"
#include "rj_control/controllers/pose_controller.hpp"
#include "rj_control/controllers/shoot_controller.hpp"
#include "rj_control/controllers/collect_controller.hpp"
#include "rj_control/controllers/pass_controller.hpp"
#include "rj_control/controllers/dribble_controller.hpp"
#include "rj_control/controllers/clear_controller.hpp"
#include "rj_control/controllers/mark_robot_controller.hpp"
#include "rj_control/controllers/rotate_controller.hpp"
#include "rj_control/barriers/barrier_certificate.hpp"
#include "rj_control/barriers/linear_cbf.hpp"

namespace control {

/**
 * @brief Controller for a given robot.
 *
 * This node is responsible for determing the command velocities for a singular robot
 * 
 */
class ControlNode : public rclcpp::Node {
public:
    /**
     * @brief Construct a new Control Node for a robot
     */
    ControlNode();

    /**
     * @brief Run the current controller
     * 
     */
    void run();

    /**
     * @brief Instantiate an instance of each of the controllers and add them to the controllers
     * unordered map
     * 
     */
    void create_controllers();
private:

    /**
     * @brief Set the current action to execute
     * 
     * @param new_action 
     */
    void set_action(action::Action new_action);

    // The robot id of the robot being controlled
    int robot_id_;
    // The current play state
    PlayState play_state_ = PlayState::halt();
    // The current action
    std::optional<action::Action> action_ = std::nullopt;
    // The count of frames where the action states it is complete
    int complete_frames_ = 0;
    // Is the current action completed
    bool completed_ = false;
    // The current world state
    std::optional<WorldState> world_state_ = std::nullopt;
    // The current field dimensions
    FieldDimensions field_dimensions_ = FieldDimensions::kDefaultDimensions;

    // The current controller being used
    uint8_t current_controller_ = rj_msgs::msg::ControllerSelect::PID_POSITION_CONTROLLER;
    // The controllers available for controlling the robot
    std::unordered_map<uint8_t, std::unique_ptr<Controller>> controllers_ = {};

    // The debug drawer
    std::optional<rj_drawing::RosDebugDrawer> drawer_ = std::nullopt;

    // Subscription to the current action for the robot
    std::shared_ptr<rclcpp::Subscription<action::Action::Msg>> action_sub_;
    // Publisher to publish when the current action has been completed
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Bool>> action_complete_pub_;
    // Subscription to the current world state
    std::shared_ptr<rclcpp::Subscription<WorldState::Msg>> world_state_sub_;
    // Subscription to the current play state
    std::shared_ptr<rclcpp::Subscription<PlayState::Msg>> play_state_sub_;
    // Subscription to the current field dimensions
    std::shared_ptr<rclcpp::Subscription<FieldDimensions::Msg>> field_dimensions_sub_;
    // Subscription to the current goalie id
    std::shared_ptr<rclcpp::Subscription<rj_msgs::msg::Goalie>> goalie_sub_;
    // Publisher to publish the commands for the robot
    std::shared_ptr<rclcpp::Publisher<ControlCommand::Msg>> setpoint_pub_;

    // Timer for scheduling the control update
    rclcpp::TimerBase::SharedPtr control_update_timer_;

    // The barrier certificate used by the controller
    std::unique_ptr<BarrierCertificate> barrier_certificate_ = {};

    // The param callback handle
    std::shared_ptr<OnSetParametersCallbackHandle> param_cb_handle_;
};

} // namespace control