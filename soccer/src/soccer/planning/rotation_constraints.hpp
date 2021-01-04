#pragma once

#include "planning/planning_params.hpp"

namespace planning {

struct RotationConstraints {
    RotationConstraints()
        : max_speed(constraints::PARAM_max_rotational_speed), max_accel(constraints::PARAM_max_rotational_accel) {}
    double max_speed;
    double max_accel;
};

} // namespace planning
