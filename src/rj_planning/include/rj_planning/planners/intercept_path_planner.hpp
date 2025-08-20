#pragma once

#include <rj_constants/constants.hpp>
#include <rj_common/planning/instant.hpp>

#include "rj_planning/planners/path_planner.hpp"
#include "rj_planning/primitives/angle_planning.hpp"
#include "rj_planning/primitives/create_path.hpp"

namespace planning {

/**
 * PathPlanner which tries to intercept the path ball as quickly as possible
 * Whether this means moving and stopping in the path of the ball
 * or completely driving through and "slapping" the ball.
 *
 * Mostly used for the goalie to block shots (w/ a target point of 0,0).
 *
 * Params taken from MotionCommand:
 *   target.position - planner will attempt to intercept ball as close to
 *                     this point as possible
 */

class InterceptPathPlanner : public PathPlanner {
public:
    InterceptPathPlanner() : PathPlanner("intercept"){};

    Trajectory plan(const PlanRequest& request) override;

    [[nodiscard]] bool is_done() const override;

private:
    // for is_done
    BallState latest_ball_state_;
    rj_geometry::Point latest_robot_pos_;
};
}  // namespace planning
