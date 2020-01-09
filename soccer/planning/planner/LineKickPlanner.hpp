#pragma once

#include <optional>
#include <Geometry2d/Point.hpp>
#include "planning/planner/MotionCommand.hpp"
#include "planning/planner/PathTargetPlanner.hpp"
#include "planning/planner/PlanRequest.hpp"
#include "planning/planner/Planner.hpp"

namespace Planning {

/**
 * Planner which plans a path to line kick a ball.
 * Uses the System State object to get the position of the ball
 * and predict its Path. It chooses the closest intersection point
 * with the ball Path it can reach in time and plans a Path so the
 * ball and robot intersect at the same time.
 *
 * TODO(ashaw596): Fix bug with replanning on real robots.
 */
class LineKickPlanner : public PlannerForCommandType<LineKickCommand> {
public:
    Trajectory plan(PlanRequest&& request) override;

    std::string name() const override { return "LinkKickPlanner"; }

private:
    PathTargetPlanner pathTargetPlanner;
};

}  // namespace Planning
