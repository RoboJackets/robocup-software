/**
 * @file pass_controller.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief Controller responsible for passing the ball directly to another robot
 * @version 0.1
 * @date 2026-01-10
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "controller.hpp"

namespace control {

class PassController : public Controller {
public:
    PassController(int robot_id, rclcpp::Node::SharedPtr control_node)
        : Controller(robot_id, std::move(control_node), "pass_controller") {}

    // Controller Methods //
    uint8_t id() override { return action::ActionType::PASS; }
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
    // The various states of the controller
    enum State {
        // Step 1: Get behind the ball
        GET_BEHIND_BALL = 0,
        // Step 2: Charge at the ball and kick
        KICK_BALL = 1,
        // Step 3: Stop moving so we don't get a double touch
        STOPPING = 2,
    };

    // The current state of the controller
    State current_state_ = State::GET_BEHIND_BALL;
};

} // namespace control