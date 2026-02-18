#pragma once

#include <rj_common/planning/instant.hpp>
#include <rj_constants/constants.hpp>
#include <rj_geometry/point.hpp>

#include "rj_planning/planners/path_planner.hpp"
#include "rj_planning/primitives/angle_planning.hpp"
#include "rj_planning/primitives/create_path.hpp"
#include "rj_planning/primitives/replanner.hpp"

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
    enum class State {
        // From start of subbehavior to the start of the slow part of the approach
        CoarseApproach,
        // Intercepts a moving ball
        Intercept,
        // Slows down the velocity of an intercepted ball
        Dampen,
        // From the slow part of the approach to the touching of the ball
        FineApproach,
    };

    CollectPathPlanner()
        : PathPlanner("collect"), average_ball_vel_(0, 0), approach_direction_(0, 0) {}

    Trajectory plan(const PlanRequest& plan_request) override;

    void reset() override;
    [[nodiscard]] bool is_done() const override;

private:
    void process_state_transition(const PlanRequest& request, BallState ball,
                                  RobotInstant* start_instant);

    void update_ball_filters(const BallState& ball);
    void update_approach_direction(const BallState& ball, const RobotInstant& start);

    Trajectory coarse_approach(
        const PlanRequest& plan_request, RobotInstant start,
        const rj_geometry::ShapeSet& static_obstacles,
        const std::vector<DynamicObstacle>& dynamic_obstacles);

    Trajectory intercept(const PlanRequest& plan_request, RobotInstant start_instant,
                         const rj_geometry::ShapeSet& static_obstacles,
                         const std::vector<DynamicObstacle>& dynamic_obstacles);

    // Dampen doesn't need to take obstacles into account.
    Trajectory dampen(const PlanRequest& plan_request, RobotInstant start_instant,
                      const rj_geometry::ShapeSet& static_obstacles,
                      const std::vector<DynamicObstacle>& dynamic_obstacles);

    Trajectory fine_approach(
        const PlanRequest& plan_request, RobotInstant start_instant,
        const rj_geometry::ShapeSet& static_obstacles,
        const std::vector<DynamicObstacle>& dynamic_obstacles);

    Trajectory invalid(const PlanRequest& plan_request,
                       const rj_geometry::ShapeSet& static_obstacles,
                       const std::vector<DynamicObstacle>& dynamic_obstacles);

    Trajectory previous_;

    State current_state_ = State::CoarseApproach;

    // Ball Velocity Filtering Variables
    rj_geometry::Point average_ball_vel_;
    bool average_ball_vel_initialized_ = false;

    rj_geometry::Point approach_direction_;

    // Intercept Target Filtering Variables
    rj_geometry::Point avg_instantaneous_intercept_target_;
    bool first_intercept_target_found_ = false;

    // Only change the target of the path if it changes significantly
    rj_geometry::Point path_intercept_target_;

    // Have we already made a dampen path
    bool path_created_for_dampen_ = false;

    // Do we have the ball in the robot
    bool is_ball_sense_ = false;

    // Threshold for switching from dampen to fine approach
    static constexpr double kDampenBallSpeedThreshold{0.75};

    // Threshold for ball velocity to try to intercept;
    static constexpr double kInterceptVelocityThreshold{0.2};

    // Threshold for chasing after the ball instead of intercepting (deg)
    static constexpr double kChaseAngleThreshold{45};
};

}  // namespace planning
