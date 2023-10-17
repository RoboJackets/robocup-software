#pragma once

#include "planning/instant.hpp"
#include "planning/planner/path_planner.hpp"
#include "planning/primitives/replanner.hpp"
#include "rj_geometry/point.hpp"

namespace planning {

/**
 * @brief PathPlanner that tries to move onto and gain control of the ball,
 * wherever the ball is. Ball MUST be slow-moving before Collect can be called.
 *
 * Params taken from MotionCommand:
 *   None
 */
class CollectPathPlanner : public PathPlanner {
public:
    enum CollectPathPathPlannerStates {
        // From start of subbehavior to the start of the slow part of the
        // approach
        CoarseApproach,
        // From the slow part of the approach to the touching of the ball
        FineApproach,
        // From touching the ball to stopped with the ball in the mouth
        Control
    };

    CollectPathPlanner()
        : PathPlanner("collect"), average_ball_vel_(0, 0), approach_direction_(0, 0) {}

    Trajectory plan(const GlobalState& global_state, const RobotIntent& robot_intent,
                    const DebugDrawer& debug_draw);

    void reset() override;
    [[nodiscard]] bool is_done() const override;

private:
    // Restarts the state machine if our calculations are whack
    // and won't intercept ball correctly anymore
    void check_solution_validity(BallState ball, RobotInstant start);

    void process_state_transition(BallState ball, RobotInstant start_instant);

    Trajectory coarse_approach(
        const PlanRequest& plan_request, RobotInstant start,
        const rj_geometry::ShapeSet& static_obstacles,
        const std::vector<DynamicObstacle>& dynamic_obstacles);

    Trajectory fine_approach(
        const PlanRequest& plan_request, RobotInstant start_instant,
        const rj_geometry::ShapeSet& static_obstacles,
        const std::vector<DynamicObstacle>& dynamic_obstacles);

    Trajectory control(const PlanRequest& plan_request, RobotInstant start,
                       const Trajectory& partial_path,
                       const rj_geometry::ShapeSet& static_obstacles,
                       const std::vector<DynamicObstacle>& dynamic_obstacles);

    Trajectory invalid(const PlanRequest& plan_request,
                       const rj_geometry::ShapeSet& static_obstacles,
                       const std::vector<DynamicObstacle>& dynamic_obstacles);

    Trajectory previous_;

    CollectPathPathPlannerStates current_state_ = CollectPathPathPlannerStates::CoarseApproach;

    // Ball Velocity Filtering Variables
    rj_geometry::Point average_ball_vel_;
    bool average_ball_vel_initialized_ = false;

    rj_geometry::Point approach_direction_;
    bool approach_direction_created_ = false;

    bool control_path_created_ = false;

    rj_geometry::Point path_coarse_target_;
    bool path_coarse_target_initialized_ = false;

    // is_done vars
    std::optional<LinearMotionInstant> cached_start_instant_;
    std::optional<rj_geometry::Point> cached_robot_pos_;
    std::optional<rj_geometry::Point> cached_ball_pos_;
};

}  // namespace planning
