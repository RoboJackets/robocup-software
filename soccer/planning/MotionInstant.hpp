#pragma once

#include <Geometry2d/Point.hpp>

namespace  Planning {
/**
 * This class represents a robot's motion "state" at a given time, including position and velocity.
 */
    struct MotionInstant {

        MotionInstant(Geometry2d::Point pos = {0, 0}, Geometry2d::Point vel = {0, 0}) : pos(pos), vel(vel) {}

        /// A position at a given point in time.
        Geometry2d::Point pos;

        /// A velocity at a given point in time.
        Geometry2d::Point vel;
    };
}
