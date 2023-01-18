#pragma once

#include "planner.hpp"

namespace planning {

/**
 * Planner which tries to intercept the path ball as quickly as possible
 * Whether this means moving and stopping in the path of the ball
 * or completely driving through and "slapping" the ball.
 *
 * Mostly used for the goalie to block shots (w/ a target point of 0,0).
 */

class InterceptPlanner : public PlannerForCommandType<InterceptMotionCommand> {
public:
    InterceptPlanner() : PlannerForCommandType<InterceptMotionCommand>("InterceptPlanner"){};

    Trajectory plan(const PlanRequest& request) override;

    [[nodiscard]] bool is_done() const override;
};
}  // namespace planning
