/**
 * @file position_controller.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief A Position controller using PID to get to a desired position
 * @version 0.1
 * @date 2026-01-04
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "controller.hpp"

#include <cmath>
#include <vector>

#include "rj_control/utilities/pid.hpp"
#include "rj_control/skills/go_to_position.hpp"

namespace control {

class PositionController : public Controller {
public:
    /**
     * @brief Construct a new Pid Controller object
     * 
     * @param robot_id The robot id to control
     * @param control_node A reference to the control node
     */
    PositionController(int robot_id, rclcpp::Node::SharedPtr control_node)
        : Controller(robot_id, std::move(control_node), "position_controller") {}

    // Controller Methods //
    uint8_t id() override { return action::ActionType::GO_TO_POINT; }
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
};

} // namespace control