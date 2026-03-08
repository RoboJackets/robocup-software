/**
 * @file rotate.hpp
 * @author Nathaniel Wert (n8.wert.b@gmail.com)
 * @brief Rotate in place to a target orientation
 * @version 0.1
 * @date 2026-01-09
 *
 * @note headings for world state are always in the range [-pi, pi]
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#pragma once

#include <cmath>

#include "rj_control/skills/skill.hpp"

namespace control {

class Rotate : public Skill<double, double> {
public:
    ControlCommand update(
        int robot_id,
        const WorldState& world_state,
        const double& heading,
        Pid& x_controller,
        Pid& y_controller,
        Pid& w_controller
    ) override;

    bool complete(
        int robot_id,
        const WorldState& world_state,
        const double& heading,
        const double& tolerance
    ) override;

private:
    /**
     * @brief Wrap an angle between [-pi, pi]
     * 
     * @param angle 
     * @return double 
     */
    static double wrap_to_pi(double angle);
};

} // namespace control