#pragma once

#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float64.hpp>
#include <spdlog/spdlog.h>
#include <fmt/core.h>

#include <rj_common/world_state.hpp>
#include <rj_common/field_dimensions.hpp>
#include <rj_control_extensions/action.hpp>
#include <rj_control_extensions/control_command.hpp>

#include <rj_msgs/msg/controller_select.hpp>

#include "rj_control/utilities/translational_pid.hpp"
#include "rj_control/utilities/rotational_pid.hpp"
#include "rj_control/skills/go_to_pose.hpp"
#include "rj_control/skills/go_to_position.hpp"
#include "rj_control/skills/rotate.hpp"

namespace control {

/**
 * @brief Controller is an abstract superclass.  All controllers should span off of
 * the controller superclass.  The main goal of a controller is to attempt to follow
 * (as best as possible) a provided plan created by the path planners.
 * 
 */
class Controller {
public:
    /**
     * @brief Construct a new Controller object
     * 
     */
    Controller(int robot_id, rclcpp::Node::SharedPtr control_node, std::string name);

    /**
     * @brief Destroy the Controller object
     * 
     */
    virtual ~Controller() = default;

    /**
     * @brief Default move constructor for the controller
     * 
     * @param other The controller moved into this controller
     */
    Controller(Controller&& other) = default;

    /**
     * @brief Default move assignment
     */
    Controller& operator=(Controller&&) = default;

    /**
     * @brief Default copy constructor for the controller
     * 
     * @param other The controller copied into this controller
     */
    Controller(const Controller& other) = default;

    /**
     * @brief Default copy assignment
     */
    Controller& operator=(const Controller&) = default;

    /**
     * @brief Get the id of the controller
     * 
     * @return uint8_t The id of the controller
     */
    virtual uint8_t id() = 0;

    /**
     * @brief Called when the controller begins operating
     * 
     */
    virtual void start(
        [[maybe_unused]] const WorldState& world_state,
        [[maybe_unused]] const FieldDimensions& field_dimensions,
        [[maybe_unused]] const action::Action& action
    ) {}

    /**
     * @brief Called when the controller stops operating
     * 
     */
    virtual void stop(
        [[maybe_unused]] const WorldState& world_state,
        [[maybe_unused]] const FieldDimensions& field_dimensions,
        [[maybe_unused]] const action::Action& action
    ) {}

    /**
     * @brief Returns true once the controller has effectively achieved the desired outcome
     * 
     * @return true 
     * @return false 
     */
    virtual bool complete(
        const WorldState& world_state,
        const FieldDimensions& field_dimensions,
        const action::Action& action
    ) = 0;

    /**
     * @brief Evaluate the controller given a robot state, world state, and a
     * trajectory to follow
     * 
     * @param world_state The current state of the world
     * @param trajectory The current trajectory to follow
     * @return MotionSetpoint The motion setpoint (x, y, w) velocity for the robot (in global frame)
     */
    virtual ControlCommand update(
        const WorldState& world_state,
        const FieldDimensions& field_dimensions,
        const action::Action& action
    ) = 0;

    /**
     * @brief Should the controller be avoiding the ball (this is passed to the barrier
     * certificate so it knows if it should be avoiding the ball)
     * 
     * @param world_state 
     * @param action 
     * @return true 
     * @return false 
     */
    virtual bool avoid_ball(
        [[maybe_unused]] const WorldState& world_state,
        [[maybe_unused]] const FieldDimensions& field_dimensions,
        [[maybe_unused]] const action::Action& action
    ) {
        return false;
    }

protected:
    void load_pid_params();

    void update_params(const std::vector<rclcpp::Parameter>& params);

    void publish_errors();

    void reset();

    // The robot id of the robot to control
    int robot_id_;
    // A reference to the control node
    rclcpp::Node::SharedPtr control_node_;

    // The pid controller for the x velocity
    TranslationalPid x_controller_;
    // The pid controller for the y velocity
    TranslationalPid y_controller_;
    // The pid controller for the w velocity
    RotationalPid w_controller_;

    // A debug subscription to reset the controller
    std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Bool>> reset_sub_;
    // A publisher to publish the x position error
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> error_x_pub_;
    // A publisher to publish the y position error
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> error_y_pub_;
    // A publisher to publish the heading error
    std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Float64>> error_heading_pub_;

    // Handle to update the parameters of the controller when they change
    std::shared_ptr<rclcpp::node_interfaces::OnSetParametersCallbackHandle> param_cb_handle_;

    // Go to pose skill
    GoToPose go_to_pose_ = GoToPose();
    // Go to position skill
    GoToPosition go_to_position_ = GoToPosition();
    // Rotate skill
    Rotate rotate_ = Rotate();
};

} // namespace control