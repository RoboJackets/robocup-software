/**
 * @file mark_robot_controller.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief The Mark Robot Controller forces a given robot to position itself a given distance from
 * an opposing robot and between the goal
 * @version 0.1
 * @date 2026-01-09
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "controller.hpp"

#include "rj_control/skills/go_to_pose.hpp"

namespace control {

class MarkRobotController : public Controller {
public:
    MarkRobotController(int robot_id, rclcpp::Node::SharedPtr control_node)
        : Controller(robot_id, std::move(control_node), "mark_robot_controller") {}

    // Controller Methods //
    uint8_t id() override { return action::ActionType::MARK_ROBOT; }
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

private:
    static rj_geometry::Pose calculate_target_pose(
        const rj_geometry::Pose& our_pose,
        const rj_geometry::Pose& their_pose,
        const FieldDimensions& field_dimensions,
        double distance
    );
};

} // namespace control