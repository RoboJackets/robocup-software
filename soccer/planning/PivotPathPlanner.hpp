#pragma once

#include "SingleRobotPathPlanner.hpp"
#include <Geometry2d/Point.hpp>

class Configuration;
class ConfigDouble;

namespace Planning {

/// Plans a path that brings the robot to the given velocity as fast as
/// possible.  Avoids obstacles.
class PivotPathPlanner : public SingleRobotPathPlanner {
public:
    virtual std::unique_ptr<Path> run(
        MotionInstant startInstant, const MotionCommand* cmd,
        const MotionConstraints& motionConstraints,
        const Geometry2d::ShapeSet* obstacles,
        std::unique_ptr<Path> prevPath = nullptr) override;

    virtual MotionCommand::CommandType commandType() const override {
        return MotionCommand::Pivot;
    }

    static void createConfiguration(Configuration* cfg);

private:
    bool shouldReplan(MotionInstant startInstant, PivotCommand command,
                      const MotionConstraints& motionConstraints,
                      const Geometry2d::ShapeSet* obstacles,
                      const Path* prevPath);
    Geometry2d::Point calculateNonblockedPathEndpoint(
        Geometry2d::Point start, Geometry2d::Point dir,
        const Geometry2d::ShapeSet* obstacles);
};

}  // namespace Planning
