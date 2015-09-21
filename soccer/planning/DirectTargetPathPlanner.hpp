#pragma once

#include "SingleRobotPathPlanner.hpp"
#include "TrapezoidalPath.hpp"

namespace Planning {

/// Simple path planner that generates a straight-line path from the start
/// instant to the goal, ignoring all obstacles.  This class will eventually be
/// replaced by something better.
class DirectTargetPathPlanner : public SingleRobotPathPlanner {
public:
    MotionCommand::CommandType commandType() const override {
        return MotionCommand::CommandType::DirectPathTarget;
    }

    virtual std::unique_ptr<Path> run(
        MotionInstant startInstant, const MotionCommand* cmd,
        const MotionConstraints& motionConstraints,
        const Geometry2d::ShapeSet* obstacles,
        std::unique_ptr<Path> prevPath = nullptr) override;

    bool shouldReplan(MotionInstant startInstant, const MotionCommand* cmd,
                      const MotionConstraints& motionConstraints,
                      const Geometry2d::ShapeSet* obstacles,
                      const Path* prevPath) const;
};

}  // namespace Planning
