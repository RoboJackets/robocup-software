#pragma once

#include "SingleRobotPathPlanner.hpp"

namespace Planning {

/// Plans a path that brings the robot to the given velocity as fast as
/// possible.  Ignores all obstacles.
///
/// Used in the following behaviors:
/// * Capture - fine approach
/// * Bump - charge towards ball
/// * LineKick - charge
/// * PassReceive - receiving state
class TargetVelPathPlanner : public SingleRobotPathPlanner {
public:
    virtual std::unique_ptr<Path> run(
        MotionInstant startInstant, MotionCommand cmd,
        const MotionConstraints& motionConstraints,
        const Geometry2d::ShapeSet* obstacles,
        std::unique_ptr<Path> prevPath = nullptr) override;

    bool shouldReplan(MotionInstant startInstant, MotionCommand cmd,
                      const MotionConstraints& motionConstraints,
                      const Geometry2d::ShapeSet* obstacles,
                      const Path* prevPath);

    virtual MotionCommand::CommandType commandType() const override {
        return MotionCommand::WorldVel;
    }
};

}  // namespace Planning
