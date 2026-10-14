#pragma once

namespace planning {

/**
 * This class contains the motion constraints that the high-level logic sets for a robot.
 *
 * The defaults are only for default-constructed values (tests, UI scaling);
 * the planner fills these from the /planner constraints.* params.
 */
struct MotionConstraints {
    double max_speed{2.0};         // m/s
    double max_acceleration{2.0};  // m/s^2
};

}  // namespace planning
