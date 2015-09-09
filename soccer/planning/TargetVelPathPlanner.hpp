#pragma once

#include "SingleRobotPathPlanner.hpp"
#include <Geometry2d/Point.hpp>

class Configuration;
class ConfigDouble;

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

    virtual MotionCommand::CommandType commandType() const override {
        return MotionCommand::WorldVel;
    }

    static void createConfiguration(Configuration* cfg);

private:
    bool shouldReplan(MotionInstant startInstant, MotionCommand cmd,
                      const MotionConstraints& motionConstraints,
                      const Geometry2d::ShapeSet* obstacles,
                      const Path* prevPath);
    Geometry2d::Point calculateNonblockedPathEndpoint(
        Geometry2d::Point start, Geometry2d::Point dir,
        const Geometry2d::ShapeSet* obstacles);

    /// If the desired target velocity changes by this much, the path is
    /// replanned
    static ConfigDouble* _targetVelChangeReplanThreshold;
};

}  // namespace Planning
