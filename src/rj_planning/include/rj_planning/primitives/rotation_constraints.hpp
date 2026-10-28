#pragma once

namespace planning {

/**
 * Rotational limits for a robot. The defaults are only for default-constructed
 * values (tests); the planner fills these from the /planner constraints.* params.
 */
struct RotationConstraints {
    double max_speed{3.0};  // rad/s
    double max_accel{3.0};  // rad/s^2
};

}  // namespace planning
