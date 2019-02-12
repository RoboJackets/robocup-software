#pragma once

#include <Geometry2d/Point.hpp>
#include "SingleRobotPathPlanner.hpp"

class Configuration;
class ConfigDouble;

namespace Planning {

/// Plans a path that brings the robot to the given velocity as fast as
/// possible.  Avoids obstacles.
class TargetVelPathPlanner : public SingleRobotPathPlanner {
public:
    TargetVelPathPlanner() : SingleRobotPathPlanner(false){};

    virtual std::unique_ptr<Path> run(PlanRequest& planRequest) override;

    virtual MotionCommand::CommandType commandType() const override {
        return MotionCommand::WorldVel;
    }

    static void createConfiguration(Configuration* cfg);

private:
    bool shouldReplan(const PlanRequest& planRequest) const;

    Geometry2d::Point calculateNonblockedPathEndpoint(
        Geometry2d::Point start, Geometry2d::Point dir,
        const Geometry2d::ShapeSet& obstacles) const;

    /// If the desired target velocity changes by this much, the path is
    /// replanned
    static ConfigDouble* _targetVelChangeReplanThreshold;
};

}  // namespace Planning
