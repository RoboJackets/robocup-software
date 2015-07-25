#pragma once

#include <Geometry2d/Point.hpp>

namespace  Planning {
/**
 * This class represents a instant on a motion. It cointains a position and velocity vector
 */
    struct MotionInstant {

        MotionInstant() : pos(), vel() {}

        MotionInstant(Geometry2d::Point pos, Geometry2d::Point vel) : pos(pos), vel(vel) {}

        /// A point on the field that the robot should use path-planning to get to
        Geometry2d::Point pos;

        /// Set the velocity in world coordinates directly (circumvents path planning)
        Geometry2d::Point vel;
    };
}