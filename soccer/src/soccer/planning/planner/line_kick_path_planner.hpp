#pragma once

#include <optional>

#include "planning/planner/collect_path_planner.hpp"
#include "planning/planner/path_planner.hpp"
#include "planning/planner/path_target_path_planner.hpp"
#include "planning/trajectory.hpp"

class Configuration;
class ConfigDouble;

namespace planning {

/**
 * PathPlanner which plans a path to line kick a ball.
 * Uses the System State object to get the position of the ball
 * and predict its Path. It chooses the closest intersection point
 * with the ball Path it can reach in time and plans a Path so the
 * ball and robot intersect at the same time.
 *
 * TODO(Kyle): Overhaul this entire planner. It's sketchy right now.
 *
 * Params taken from MotionCommand:
 *   target.position - planner will kick to this point
 */
class LineKickPathPlanner : public PathPlanner {
public:
    LineKickPathPlanner() : PathPlanner("line_kick"){};
    Trajectory plan(const PlanRequest& plan_request) override;

    void reset() override {
        prev_path_ = {};
        target_kick_pos_ = std::nullopt;
        current_state_ = INITIAL_APPROACH;
        average_ball_vel_initialized_ = false;
        has_created_plan = false;
    }
    [[nodiscard]] bool is_done() const override;

private:
    enum State { INITIAL_APPROACH, FINAL_APPROACH };
    State current_state_ = INITIAL_APPROACH;
    PathTargetPathPlanner path_target_{};
    CollectPathPlanner collect_planner_{};
    Trajectory prev_path_;

    // These constants could be tuned more
    static constexpr double kIsDoneBallVel = 1.5;
    static constexpr double kFinalRobotSpeed = 1.0;
    static constexpr double kPredictIn = 0.5; // seconds
    static constexpr double kAvoidBallBy = 0.05;

    rj_geometry::Point average_ball_vel_;
    bool average_ball_vel_initialized_ = false;
    bool has_created_plan = false;
    std::optional<rj_geometry::Point> target_kick_pos_;

    // Trajectory initial(BallState ball, MotionCommand command, RobotInstant start_instant,
    // ShapeSet static_obstacles, std::vector<DynamicObstacle> dynamic_obstacles);
    Trajectory initial(const PlanRequest& plan_request);
    Trajectory final(const PlanRequest& plan_request);
    // Trajectory final(BallState ball, MotionCommand command, RobotInstant start_instant, ShapeSet
    // static_obstacles, std::vector<DynamicObstacle> dynamic_obstacles);
    void process_state_transition();

    // PlayState::State current_state_;
};

}  // namespace planning
