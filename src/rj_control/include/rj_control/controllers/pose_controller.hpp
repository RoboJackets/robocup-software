/**
 * @file pose_controller.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief A Pose controller using PID to get to a desired pose
 * @version 0.1
 * @date 2026-01-04
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <cmath>
#include <vector>

#include "rj_control/controllers/controller.hpp"
#include "rj_control/utilities/pid.hpp"
#include "rj_control/skills/go_to_pose.hpp"

namespace control {

/**
 * @brief The PID Pose controller attempts to meet a given Pose using an x, y, and theta pid
 * controller
 * 
 */
class PoseController : public Controller {
public:
    /**
     * @brief Construct a new Pose Controller
     * 
     * @param robot_id The robot to control
     * @param control_node The control node
     */
    PoseController(int robot_id, rclcpp::Node::SharedPtr control_node)
        : Controller(robot_id, std::move(control_node), "pose_controller") {}

    // Controller Methods //
    uint8_t id() override { return action::ActionType::GO_TO_POSE; }
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