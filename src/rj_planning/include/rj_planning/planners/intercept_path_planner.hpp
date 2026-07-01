#pragma once

#include <rj_common/planning/instant.hpp>
#include <rj_constants/constants.hpp>

#include "rj_planning/planners/path_planner.hpp"
#include "rj_planning/primitives/angle_planning.hpp"
#include "rj_planning/primitives/create_path.hpp"

namespace planning {

/**
 * PathPlanner which tries to intercept the ball along its path as close to the
 * ball's current position as possible.
 *
 * The planner samples points along the ball's predicted path (restricted to our
 * defense area / the goalie box) and picks the earliest one (closest to the
 * ball's current position) that the robot can beat the ball to, then paths to
 * that point. If the robot cannot beat the ball to any such point, it instead
 * aims for the point on the ball's path closest to the robot (the perpendicular
 * projection), to get as close to the ball's path as possible.
 *
 * Mostly used for the goalie to block shots.
 *
 * Takes no parameters from MotionCommand.
 */

class InterceptPathPlanner : public PathPlanner {
public:
    InterceptPathPlanner() : PathPlanner("intercept"){};

    Trajectory plan(const PlanRequest& request) override;

    [[nodiscard]] bool is_done() const override;

private:
    // Number of points sampled along the ball's path when searching for an
    // interception point.
    static constexpr int kNumSamples = 20;

    // for is_done
    BallState latest_ball_state_;
    rj_geometry::Point latest_robot_pos_;
};
}  // namespace planning
