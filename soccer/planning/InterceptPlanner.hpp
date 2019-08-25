#pragma once

#include "DirectTargetPathPlanner.hpp"
#include "SingleRobotPathPlanner.hpp"

class Configuration;
class ConfigDouble;

namespace Planning {

/**
 * Planner which tries to intercept the path ball as quickly as possible
 * Whether this means moving and stopping in the path of the ball
 * or completely driving through and "slapping" the ball.
 *
 * Mostly used for the goalie / defenders to block shots
 */

class InterceptPlanner : public SingleRobotPathPlanner {
public:
    InterceptPlanner()
        : SingleRobotPathPlanner(false),
          directPlanner(),
          prevPathTarget(0, 0){};

    virtual std::unique_ptr<Path> run(PlanRequest& planRequest) override;

    virtual MotionCommand::CommandType commandType() const override {
        return MotionCommand::Intercept;
    }

private:
    DirectTargetPathPlanner directPlanner;
    Geometry2d::Point prevPathTarget;
};
}  // namespace Planning
