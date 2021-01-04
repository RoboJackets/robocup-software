#pragma once

#include "planning/planning_params.hpp"

namespace planning {

/**
 * This class contains the motion constraints that the high-level logic sets for a robot.
 */
struct MotionConstraints {
    MotionConstraints()
        : max_speed(constraints::PARAM_max_translational_speed),
          max_acceleration(constraints::PARAM_max_translational_accel) {}

    double max_speed;
    double max_acceleration;
};

} // namespace planning