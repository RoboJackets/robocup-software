#pragma once

#include "SingleRobotPathPlanner.hpp"

namespace Planning {

/// Plans a path that accelerates the bot form its current velocity to the
/// target velocity.  Ignores all obstacles.

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

    virtual MotionCommand::CommandType commandType() const override {
        return MotionCommand::WorldVel;
    }
};

}  // namespace Planning
