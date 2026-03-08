/**
 * @file go_to_pose.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief State responsible for getting to a given pose
 * @version 0.1
 * @date 2026-01-08
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <utility>

#include "rj_control/skills/skill.hpp"

namespace control {

class GoToPose : public Skill<rj_geometry::Pose, std::pair<double, double>> {
public:
    ControlCommand update(
        int robot_id,
        const WorldState& world_state,
        const rj_geometry::Pose& command,
        Pid& x_controller,
        Pid& y_controller,
        Pid& w_controller
    ) override;

    bool complete(
        int robot_id,
        const WorldState& world_state,
        const rj_geometry::Pose& command,
        const std::pair<double, double>& tolerance
    ) override;
};

} // namespace control