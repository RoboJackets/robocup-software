/**
 * @file collect_controller.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief A Controller that drives the robot towards the ball turning on the dribbler.  This controller
 * considers done as when the ball is very close and/or we have ball sense
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

/**
 * @brief The Collect Controller attempts to drive towards the ball and turn on the dribbler.  The 
 * collect controller considers done as having the ball with ball sense and will continue to collect the
 * ball as long as it is running
 * 
 */
class CollectController : public Controller {
public:
    /**
     * @brief Construct a new Collect Controller
     * 
     * @param robot_id The robot to control
     * @param control_node The control node
     */
    CollectController(int robot_id, rclcpp::Node::SharedPtr control_node)
        : Controller(robot_id, std::move(control_node), "collect_controller") {}

    // Controller Methods //
    uint8_t id() override { return action::ActionType::COLLECT; }
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