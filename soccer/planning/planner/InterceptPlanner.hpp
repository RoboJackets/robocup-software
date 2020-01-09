#pragma once

#include "planning/planner/Planner.hpp"
#include "planning/planner/PathTargetPlanner.hpp"
#include "planning/planner/MotionCommand.hpp"

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
    Trajectory plan(PlanRequest&& request) override;
    std::string name() const override {return "InterceptPlanner";}

private:
    PathTargetPlanner pathTargetPlanner;
    Geometry2d::Point prevPathTarget;
};
}  // namespace Planning
