#include "planning/planner/goalie_idle_planner.hpp"

namespace planning {

Trajectory GoalieIdlePlanner::plan(const PlanRequest& plan_request) {
    Trajectory temp_trajectory;
    return temp_trajectory;
}

void GoalieIdlePlanner::reset() {}

bool GoalieIdlePlanner::is_done() const { return false; }

}  // namespace planning
