#pragma once

#include "Planner.hpp"
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

class InterceptPlanner : public PlannerForCommandType<InterceptCommand> {
public:
    InterceptPlanner(): PlannerForCommandType<InterceptCommand>("InterceptPlanner"){};

    Trajectory plan(PlanRequest&& request) override;

private:
    Geometry2d::Point _prevTargetPoint;
};
}  // namespace Planning
