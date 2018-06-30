#pragma once

#include "RRTPlanner.hpp"
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
    InterceptPlanner() : SingleRobotPathPlanner(false), rrtPlanner(0, 250){};
    virtual std::unique_ptr<Path> run(PlanRequest& planRequest) override;

    virtual MotionCommand::CommandType commandType() const override {
        return MotionCommand::Intercept;
    }

private:
    bool shouldReplan(const PlanRequest& planRequest) const;

    RRTPlanner rrtPlanner;
    Geometry2d::Point targetInterceptPos;
    int reusePathCount = 0;
};

} // namespace Planning