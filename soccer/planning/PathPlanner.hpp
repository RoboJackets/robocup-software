#pragma once

#include <planning/MotionConstraints.hpp>
#include <planning/MotionInstant.hpp>
#include <planning/Path.hpp>

namespace Planning {
/**
 * @brief Interface for Path Planners
 */
class PathPlanner {
public:
    /**
     * Returns an obstacle-free Path subject to the specified MotionContraints.
     */
    virtual std::unique_ptr<Path> run(
        MotionInstant startInstant, MotionInstant endInstant,
        const MotionConstraints& motionConstraints,
        const Geometry2d::CompositeShape* obstacles) = 0;
};
}
