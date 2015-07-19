#pragma once
#include "planning/Path.hpp"
#include "planning/PathPlanner.hpp"

namespace Planning {
    class DirectPathPlanner : public PathPlanner {
    public:
        virtual std::unique_ptr<Path> run(
                MotionInstant startInstant,
                MotionInstant motionCommand,
                const MotionConstraints &motionConstraints,
                const Geometry2d::CompositeShape* obstacles) override;
    };
}