#pragma once

#include "planning/instant.hpp"
#include "planning/planner/planner.hpp"
#include "planning/primitives/replanner.hpp"

class Configuration;
class ConfigDouble;

namespace planning {

/**
 * @brief Planner which tries to get a fast-moving ball into its mouth to slow
 * it down. Paired with CollectPlanner, this forms the basis of receiving
 * passes.
 *
 * Params taken from MotionCommand:
 *   target.position - the direction we will try and bounce the ball while
 *                     slowing it down to speed up actions after capture
 */
class SettlePlanner : public Planner {
public:
    enum class SettlePlannerStates {
        // Moves to the ball path in front of it
        Intercept,
        // Starts to dampen the ball with backward motion
        Dampen
    };

    SettlePlanner()
        : Planner("settle"), avg_instantaneous_intercept_target_(0, 0), average_ball_vel_(0, 0) {}

    Trajectory plan(const PlanRequest& plan_request) override;

    void reset() override;
    [[nodiscard]] bool is_done() const override;

    static void create_configuration(Configuration* cfg);

private:
    // Given the global target in `target_bounce_direction`
    // Calculate the delta position to get the robot in the correct location
    // And the face point to get the bounce right
    // If no target_bounce_direction is given, just get in front and face the ball
    void calc_delta_pos_for_dir(BallState ball, RobotInstant start_instant,
                            double* angle_out, rj_geometry::Point* delta_robot_pos,
                            rj_geometry::Point* face_pos);

    // Restarts the state machine if our calculations are whack
    // and won't intercept ball correctly anymore
    void check_solution_validity(BallState ball, RobotInstant start_instant,
                               rj_geometry::Point delta_pos);

    // Figures out when to move to each state
    // (only in the standard transition)
    //
    // Note: start_instant may be changed inside this function
    //       when we are basically at the correct location and
    //       need to start the dampen
    void process_state_transition(BallState ball, RobotInstant* start_instant,
                                double angle, rj_geometry::Point delta_pos);

    // State functions
    Trajectory intercept(const PlanRequest& plan_request,
                         RobotInstant start_instant,
                         const rj_geometry::ShapeSet& static_obstacles,
                         const std::vector<DynamicObstacle>& dynamic_obstacles,
                         rj_geometry::Point delta_pos, rj_geometry::Point face_pos);

    // Dampen doesn't need to take obstacles into account.
    Trajectory dampen(const PlanRequest& plan_request, RobotInstant start_instant,
                      rj_geometry::Point delta_pos, rj_geometry::Point face_pos);

    Trajectory invalid(const PlanRequest& plan_request,
                       const rj_geometry::ShapeSet& static_obstacles,
                       const std::vector<DynamicObstacle>& dynamic_obstacles);

    std::optional<rj_geometry::Point> target_bounce_direction_;

    SettlePlannerStates current_state_ = SettlePlannerStates::Intercept;

    // Intercept Target Filtering Variables
    rj_geometry::Point avg_instantaneous_intercept_target_;
    rj_geometry::Point average_ball_vel_;
    bool first_intercept_target_found_ = false;
    bool first_ball_vel_found_ = false;

    // Only change the target of the path if it changes significantly
    rj_geometry::Point path_intercept_target_;

    bool path_created_for_dampen_ = false;

    Trajectory previous_;
};
}  // namespace planning
