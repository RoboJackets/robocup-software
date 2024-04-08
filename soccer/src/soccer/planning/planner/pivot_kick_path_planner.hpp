#pragma once

#include <optional>

#include "planning/planner/collect_path_planner.hpp"
#include "planning/planner/path_planner.hpp"
#include "planning/planner/path_target_path_planner.hpp"
#include "planning/trajectory.hpp"

namespace planning {

/**
 * PathPlanner which plans a path to pivot kick a ball.
 * code stolen from line kick planner
 * write the rest of this later
 *
 * Params taken from MotionCommand:
 *   target.position - planner will kick to this point
 */
class PivotKickPathPlanner : public PathPlanner {
public:
    PivotKickPathPlanner() : PathPlanner("pivot_kick"){};
    Trajectory plan(const PlanRequest& plan_request) override;

    void reset() override {
        prev_path_ = {};
        current_state_ = INITIAL_APPROACH;
        average_ball_vel_initialized_ = false;
    }

    [[nodiscard]] bool is_done() const override;

private:
    enum State { INITIAL_APPROACH, FINAL_APPROACH };

    State current_state_{INITIAL_APPROACH};

    PathTargetPathPlanner path_target_{};
    CollectPathPlanner collect_planner_{};
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
     * Returns the trajectory during the initial stage.
     * Uses PathTargetPathPlanner to draw a path to the spot to kick from.
     * Avoids the ball
     */
    Trajectory initial(const PlanRequest& plan_request);

    /**
     * Returns the trajectory during the final stage.
     * Uses PathTargetPathPlanner to draw a path directly into the ball.
     * Tries to hit the ball with the mouth of the robot.
     */
    Trajectory final(const PlanRequest& plan_request);

    /**
     * Decides if the intial approach is complete and updates internal state as necessary.
     */
    void process_state_transition();
};

}  // namespace planning
