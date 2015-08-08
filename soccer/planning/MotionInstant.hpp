#pragma once

#include <Geometry2d/Point.hpp>

namespace  Planning {
/**
 * This class represents a instant on a motion. It cointains a position and velocity vector
 */
    struct MotionInstant {

        MotionInstant() : pos(), vel() {}

        MotionInstant(Geometry2d::Point pos, Geometry2d::Point vel) : pos(pos), vel(vel) {}

        /// A position at a given point in time.
        Geometry2d::Point pos;

        /// A velocity at a given point in time.
        Geometry2d::Point vel;
    };
}