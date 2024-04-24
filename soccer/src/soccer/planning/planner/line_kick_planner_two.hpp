#pragma once

#include <optional>

#include "planning/planner/path_planner.hpp"
#include "planning/planner/path_target_path_planner.hpp"
#include "planning/trajectory.hpp"

namespace planning {

/**
 * PathPlanner which plans a path to line kick a ball.
 * It takes the ball's current trajectory and plans a path for the robot
 * to intersect with the ball.
 *
 * It is in two stages. The first stage drives up to the ball, trying to avoid
 * accidentally tapping it out of place. The second stage drives directly
 * into the ball, hopefully kicking it if the hardware has been instructed
 * to kick.
 *
 * Because of the two-stage system, this planner is *stateful*. It ought
 * not to be destructed and re-constructed during a single execution;
 * the best approach is to call plan() on each tick.
 * However, it also shouldn't *completely* break if it is reset.
 *
 * Params taken from MotionCommand:
 *   target.position - planner will kick to this point
 */
class LineKickPlannerTwo : public PathPlanner {
public:
    LineKickPlannerTwo() : PathPlanner("line_kick_two"){};
    Trajectory plan(const PlanRequest& plan_request) override;

    void reset() override {
        prev_path_ = {};
        average_ball_vel_initialized_ = false;
    }

    [[nodiscard]] bool is_done() const override;

private:
    PathTargetPathPlanner path_target_{};
    Trajectory prev_path_;

    // These constants could be tuned more
    static constexpr double kIsDoneBallVel{1.5};
    static constexpr double kFinalRobotSpeed{1.0};
    static constexpr double kPredictIn{0.5};  // seconds
    static constexpr double kAvoidBallBy{0.05};
    static constexpr double kLowPassFilterGain{0.2};

    rj_geometry::Point average_ball_vel_;
    bool average_ball_vel_initialized_ = false;

    /**
     * Returns the trajectory during the final stage.
     * Uses PathTargetPathPlanner to draw a path directly into the ball.
     * Tries to hit the ball with the mouth of the robot.
     */
    Trajectory final(const PlanRequest& plan_request);
};

}  // namespace planning
