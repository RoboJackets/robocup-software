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
    PivotPathPlanner() : SingleRobotPathPlanner(false){};
    virtual std::unique_ptr<Path> run(SinglePlanRequest &planRequest) override;

    virtual MotionCommand::CommandType commandType() const override {
        return MotionCommand::Pivot;
    }

    static void createConfiguration(Configuration* cfg);

private:
    bool shouldReplan(const SinglePlanRequest &planRequest) const;

    static ConfigDouble* _pivotRadius;
};

}  // namespace Planning
