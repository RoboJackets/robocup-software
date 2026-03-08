/**
 * @file clear_controller.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief The clear controller is most likely to be executed by the goalie, the main idea
 * is that the goalie will get the ball in its possession, rotate, and set off the chipper
 * In the direction of the clear target
 * @version 0.1
 * @date 2026-01-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "controller.hpp"

namespace control {

class ClearController : public Controller {
public:
    ClearController(int robot_id, rclcpp::Node::SharedPtr control_node)
        : Controller(robot_id, std::move(control_node), "clear_controller") {}

    // Controller Methods //
    uint8_t id() override { return action::ActionType::CLEAR; }
    void start(
        const WorldState& world_state,
        const FieldDimensions& field_dimensions,
        const action::Action& action
    ) override;
    bool complete(
        const WorldState& world_state,
        const FieldDimensions& field_dimensions,
        const action::Action& action
    ) override;
    ControlCommand update(
        const WorldState& world_state,
        const FieldDimensions& field_dimensions,
        const action::Action& action
    ) override;
    bool avoid_ball(
        const WorldState& world_state,
        const FieldDimensions& field_dimensions,
        const action::Action& action
    ) override;
    // End Controller Methods //

private:
    // The current state of the controller
    enum State {
        // The ball is outside of the goal area so we are positioning ourselves between the ball and the goal
        INTERCEPTING = 0,
        // The ball is inside the goal area so we are collecting it
        COLLECTING = 1,
        // We have the ball so we are rotating to clear it
        ROTATING = 2,
        // We are clearing the ball
        CLEARING = 3,
    };

    // The current state
    State current_state_ = State::INTERCEPTING;
};

} // namespace control