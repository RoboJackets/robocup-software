#pragma once

#include <spdlog/spdlog.h>

#include <rj_common/planning/instant.hpp>
#include <rj_constants/constants.hpp>
#include <rj_geometry/point.hpp>

#include "rj_planning/planners/path_planner.hpp"
#include "rj_planning/primitives/angle_planning.hpp"
#include "rj_planning/primitives/create_path.hpp"
#include "rj_planning/primitives/replanner.hpp"
#include "rj_planning/primitives/rrt_util.hpp"

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
        COARSE_APPROACH,
        // Intercepts a moving ball
        INTERCEPT,
        // Slows down the velocity of an intercepted ball
        DAMPEN,
        // From the slow part of the approach to the touching of the ball
        FINE_APPROACH,
    };

    CollectPathPlanner()
        : PathPlanner("collect"), average_ball_vel_(0, 0), approach_direction_(0, 0) {}

    Trajectory plan(const PlanRequest& plan_request) override;

    void reset() override;
    [[nodiscard]] bool is_done() const override;

private:
    // Restarts the state machine if our calculations are whack
    // and won't intercept ball correctly anymore
    void check_solution_validity(BallState ball, RobotInstant start);

    [[nodiscard]] BallState get_active_ball(const PlanRequest& request);

    [[nodiscard]] bool has_vision_ball_sense(const BallState& observed_ball,
                                             const RobotInstant& start_instant) const;

    void process_state_transition(const PlanRequest& request, BallState ball,
                                  RobotInstant* start_instant);

    Trajectory coarse_approach(const PlanRequest& plan_request, RobotInstant start,
                               const ObstacleSet& obstacles);

    Trajectory intercept(const PlanRequest& plan_request, RobotInstant start_instant,
                         const ObstacleSet& obstacles);

    // Dampen doesn't need to take obstacles into account.
    Trajectory dampen(const PlanRequest& plan_request, RobotInstant start_instant,
                      const ObstacleSet& obstacles);

    Trajectory fine_approach(const PlanRequest& plan_request, RobotInstant start_instant,
                             const ObstacleSet& obstacles);

    Trajectory invalid(const PlanRequest& plan_request, const ObstacleSet& obstacles);

    Trajectory previous_;

    CollectPathPathPlannerStates current_state_ = CollectPathPathPlannerStates::COARSE_APPROACH;

    // Ball Velocity Filtering Variables
    rj_geometry::Point average_ball_vel_;
    bool average_ball_vel_initialized_ = false;

    rj_geometry::Point approach_direction_;

    rj_geometry::Point path_coarse_target_;
    bool path_coarse_target_initialized_ = false;

    // Intercept Target Filtering Variables
    rj_geometry::Point avg_instantaneous_intercept_target_;
    bool first_intercept_target_found_ = false;

    // Only change the target of the path if it changes significantly
    rj_geometry::Point path_intercept_target_;

    // Have we already made a dampen path
    bool path_created_for_dampen_ = false;

    // Do we have the ball in the robot
    bool is_ball_sense_ = false;

    // Toggle the vision-based fallback for simulated ball sense during collect.
    bool visual_ball_sense_ = true;

    // Use a short-lived cached ball estimate to bridge the vision dropout that
    // happens when the ball enters the mouth.
    BallState active_ball_;
    BallState last_visible_ball_;
    bool last_visible_ball_initialized_ = false;

    static constexpr RJ::Seconds kVisionBallRetentionWindow{0.25};

    // Threshold for switching from dampen to fine approach
    static constexpr double kDampenBallSpeedThreshold{0.75};

    // Threshold for ball velocity to try to intercept;
    static constexpr double kInterceptVelocityThreshold{0.2};

    // Threshold for chasing after the ball instead of intercepting (deg)
    static constexpr double kChaseAngleThreshold{45};
};

}  // namespace planning
