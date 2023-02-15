#include "planning/planner/penalty_kick_planner.hpp"

namespace planning { 

Trajectory PenaltyKickPlanner::plan(const PlanRequest& plan_request) {
    // lots of this is duplicated from PathTargetPlanner, because there's not
    // an easy way to convert from one PlanRequest to another
    
    // TODO: make planners composable, rather than relying
    // on the stupid *MotionCommand.msg paradigm
    
    // (1) dribble the ball until close to the goal (legal in PK)
    
    // (2) shoot ball at open side of goal
}

void PenaltyKickPlanner::reset() {}

bool PenaltyKickPlanner::is_done() const { return false; }

} // namespace planning
