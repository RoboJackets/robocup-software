#pragma once

#include "planning/Instant.hpp"
#include "planning/planner/Planner.hpp"
#include "planning/primitives/Replanner.hpp"

class Configuration;
class ConfigDouble;

namespace Planning {

/**
 * @brief Planner which tries to move around the ball to intercept it
 */
class SettlePlanner : public PlannerForCommandType<SettleCommand> {
public:
    enum class SettlePlannerStates {
        // Moves to the ball path in front of it
        Intercept,
        // Starts to dampen the ball with backward motion
        Dampen
    };

    SettlePlanner()
        : PlannerForCommandType("settle"),
          avg_instantaneous_intercept_target_(0, 0),
          average_ball_vel_(0, 0) {}

    Trajectory plan(const PlanRequest& plan_request) override;

    void reset() override;

    static void create_configuration(Configuration* cfg);

private:
    // Given the global target in `target_bounce_direction`
    // Calculate the delta position to get the robot in the correct location
    // And the face point to get the bounce right
    // If no target_bounce_direction is given, just get in front and face the ball
    void calc_delta_pos_for_dir(BallState ball, RobotInstant start_instant,
                            double* angle_out, Geometry2d::Point* delta_robot_pos,
                            Geometry2d::Point* face_pos);

    // Restarts the state machine if our calculations are whack
    // and won't intercept ball correctly anymore
    void check_solution_validity(BallState ball, RobotInstant start_instant,
                               Geometry2d::Point delta_pos);

    // Figures out when to move to each state
    // (only in the standard transition)
    //
    // Note: start_instant may be changed inside this function
    //       when we are basically at the correct location and
    //       need to start the dampen
    void process_state_transition(BallState ball, RobotInstant* start_instant,
                                double angle, Geometry2d::Point delta_pos);

    // State functions
    Trajectory intercept(const PlanRequest& plan_request,
                         RobotInstant start_instant,
                         const Geometry2d::ShapeSet& static_obstacles,
                         const std::vector<DynamicObstacle>& dynamic_obstacles,
                         Geometry2d::Point delta_pos, Geometry2d::Point face_pos);

    // Dampen doesn't need to take obstacles into account.
    Trajectory dampen(const PlanRequest& plan_request, RobotInstant start_instant,
                      Geometry2d::Point delta_pos, Geometry2d::Point face_pos);

    Trajectory invalid(const PlanRequest& plan_request,
                       const Geometry2d::ShapeSet& static_obstacles,
                       const std::vector<DynamicObstacle>& dynamic_obstacles);

    std::optional<Geometry2d::Point> target_bounce_direction_;

    SettlePlannerStates current_state_ = SettlePlannerStates::Intercept;

    // Intercept Target Filtering Variables
    Geometry2d::Point avg_instantaneous_intercept_target_;
    Geometry2d::Point average_ball_vel_;
    bool first_intercept_target_found_ = false;
    bool first_ball_vel_found_ = false;

    // Only change the target of the path if it changes significantly
    Geometry2d::Point path_intercept_target_;

    bool path_created_for_dampen_ = false;

    Trajectory previous_;

    // How much of the ball seed to contact the ball with
    // before slowing down to dampen the initial hit
    static ConfigDouble* ball_speed_percent_for_dampen;  // %

    // Closest dist to start searching for intercept points
    static ConfigDouble* search_start_dist;  // m
    // Furthest dist to search for intercept points
    static ConfigDouble* search_end_dist;  // m
    // What dist increment to search for intercepts
    static ConfigDouble* search_inc_dist;  // m

    // How much sooner should we reach the intercept point than we need to
    // This is a percent of the calculated intercept time
    // Numbers greater than 1 mean we increase intercept time needed by X% over
    // actual Numbers less than 1 mean we get there X% faster than we plan
    // (Shouldn't ever happen)
    static ConfigDouble* intercept_buffer_time;  // %

    // Gain on the averaging function to smooth the target point to intercept
    // This is due to the high flucations in the ball velocity frame to frame
    // a*new_point + (1-a)*old_point
    // The lower the number, the less noise affects the system, but the slower
    // it responds to changes The higher the number, the more noise affects the
    // system, but the faster it responds to changes
    static ConfigDouble* target_point_gain;

    // Gain on the averaging function to smooth the ball velocity to for any
    // motion commands This is due to the high flucations in the ball velocity
    // frame to frame a*new_point + (1-a)*old_point The lower the number, the less
    // noise affects the system, but the slower it responds to changes The
    // higher the number, the more noise affects the system, but the faster it
    // responds to changes
    static ConfigDouble* ball_vel_gain;

    // Distance between robot and closest point on ball line such that we move
    // directly into the ball line instead of trying to find the point we hit
    // first This does take into account slow moving balls in which we should
    // move onto the ball to capture it
    static ConfigDouble* shortcut_dist;  // m

    // If the ball velocity angle changes by a large amount
    // we want to quickly react and clear all the smoothing filters
    // Lower numbers means it reacts faster, but more chance for false positives
    // Higher numbers means slower reaction, but less false positives
    static ConfigDouble* max_ball_angle_for_reset;  // Deg

    // If the ball velocity itself changes by a large amount
    // we want to quickly react and clear all the smoothing filters
    // Lower numbers means it reacts faster, but more chance for false positives
    // Higher numbers means slower reaction, but less false positives
    static ConfigDouble* max_ball_vel_for_path_reset;  // m/s

    // Max angle between ball and target bounce direction
    static ConfigDouble* max_bounce_angle;  // Deg
};
}  // namespace Planning
