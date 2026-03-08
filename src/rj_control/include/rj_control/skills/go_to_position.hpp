/**
 * @file go_to_position.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief State responsible for getting to a given position (ignoring orientation)
 * @version 0.1
 * @date 2026-01-08
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include "rj_control/skills/skill.hpp"

namespace control {

class GoToPosition : public Skill<rj_geometry::Point, double> {
public:
    ControlCommand update(
        int robot_id,
        const WorldState& world_state,
        const rj_geometry::Point& command,
        Pid& x_controller,
        Pid& y_controller,
        Pid& w_controller
    ) override;

    bool complete(
        int robot_id,
        const WorldState& world_state,
        const rj_geometry::Point& command,
        const double& tolerance
    ) override;
};

} // namespace control