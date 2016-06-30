#pragma once

#include "SingleRobotPathPlanner.hpp"
#include <Geometry2d/Point.hpp>

class Configuration;
class ConfigDouble;

namespace Planning {

/// Plans a path that pivots around and kicks a stationary ball.
class PivotPathPlanner : public SingleRobotPathPlanner {
public:
    PivotPathPlanner() : SingleRobotPathPlanner(false){};
    virtual std::unique_ptr<Path> run(SinglePlanRequest& planRequest) override;

    virtual MotionCommand::CommandType commandType() const override {
        return MotionCommand::Pivot;
    }

    static void createConfiguration(Configuration* cfg);

private:
    bool shouldReplan(const SinglePlanRequest& planRequest) const;

    static ConfigDouble* _pivotRadiusMultiplier;
};
}  // namespace Planning
