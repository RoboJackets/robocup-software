#pragma once

namespace planning {

/**
 * This class contains the motion constraints that the high-level logic sets for a robot.
 */
struct MotionConstraints {
    MotionConstraints() : max_speed(2.0), max_acceleration(2.0) {}

    double max_speed;
    double max_acceleration;
};

}  // namespace planning