/**
 * @file rotate_controller.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief The rotate controller will rotate to a given heading
 * @version 0.1
 * @date 2026-01-10
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "controller.hpp"

namespace control {

class RotateController : public Controller {
public:
    RotateController(int robot_id, rclcpp::Node::SharedPtr control_node)
        : Controller(robot_id, std::move(control_node), "rotate_controller") {};

    // Controller Methods //
    uint8_t id() override { return action::ActionType::ROTATE_TO_HEADING; }
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
    // End Controller Methods //
};

} // namespace control