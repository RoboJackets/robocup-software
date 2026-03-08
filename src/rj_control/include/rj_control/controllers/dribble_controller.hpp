/**
 * @file dribble_controller.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief The dribble controller will attempt to get the ball and take it
 * to a given pose on the field
 * @version 0.1
 * @date 2026-01-04
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "rj_control/controllers/controller.hpp"
#include "rj_control/utilities/pid.hpp"
#include "rj_control/skills/go_to_pose.hpp"

namespace control {

class DribbleController : public Controller {
public:
    DribbleController(int robot_id, rclcpp::Node::SharedPtr control_node)
        : Controller(robot_id, std::move(control_node), "dribble_controller") {};

    // Controller Methods //
    uint8_t id() override { return action::ActionType::DRIBBLE; }
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
    // End Controller Method //

private:
    /**
     * @brief The different states
     * 
     */
    enum State {
        // Step 1: get behind ball
        GET_BEHIND_BALL = 0,
        // Step 2: move with ball to target
        MOVE_WITH_BALL = 1,
    };

    // The current state
    State current_state_ = State::GET_BEHIND_BALL;
};

} // namespace control