#include "planning_params.hpp"

namespace planning {

DEFINE_FLOAT64(kPlanningParamModule, timeout, 0.1, "Timeout for vision data to reach planning (s)");

DEFINE_NS_FLOAT64(kPlanningParamModule, constraints, max_translational_speed, 8.0,
                  "Maximum translational speed for planning (m/s)");
DEFINE_NS_FLOAT64(kPlanningParamModule, constraints, max_translational_accel, 5.0,
                  "Maximum translational acceleration for planning (m/s^2)");
DEFINE_NS_FLOAT64(kPlanningParamModule, constraints, max_rotational_speed, 1.0,
                  "Maximum rotational speed for planning (rad/s)");
DEFINE_NS_FLOAT64(kPlanningParamModule, constraints, max_rotational_accel, 1.0,
                  "Maximum rotational acceleration for planning (rad/s^2)");

DEFINE_NS_FLOAT64(
    kPlanningParamModule, replanner, pos_change_threshold, 0.025,
    "Position target threshold (m); if the change in position from the originally planned "
    "target is smaller than this amount no replanning will occur.");
DEFINE_NS_FLOAT64(
    kPlanningParamModule, replanner, vel_change_threshold, 0.025,
    "Velocity target threshold (m/s); if the change in velocity from the originally planned "
    "target is smaller than this amount no replanning will occur.");
DEFINE_NS_FLOAT64(
    kPlanningParamModule, replanner, partial_replan_lead_time, 0.1,
    "Portion of the path (seconds) that will not be changed in the case of a partial replan");
DEFINE_NS_FLOAT64(
    kPlanningParamModule, replanner, off_path_threshold, 0.1,
    "Position error threshold (m), a partial replan will be forced if we are not within this "
    "amount of the planned trajectory.");

DEFINE_NS_BOOL(kPlanningParamModule, rrt, enable_debug_drawing, false,
               "Whether to enable RRT debug drawing");
DEFINE_NS_FLOAT64(kPlanningParamModule, rrt, step_size, 0.15, "Step size for RRT (m)");
DEFINE_NS_FLOAT64(kPlanningParamModule, rrt, goal_bias, 0.3,
                  "Chance that the RRT will extend directly towards the goal (unitless)");
DEFINE_NS_FLOAT64(kPlanningParamModule, rrt, waypoint_bias, 0.5,
                  "Chance that the RRT will extend directly towards a waypoint (unitless)");
DEFINE_NS_INT64(kPlanningParamModule, rrt, min_iterations, 50,
                "Minimum number of RRT iterations to run (unused without RRT* enabled)");
DEFINE_NS_INT64(kPlanningParamModule, rrt, max_iterations, 500,
                "Maximum number of RRT iterations to run before giving up");

DEFINE_NS_FLOAT64(
    kPlanningParamModule, escape, step_size, 0.1,
    "Step size for the RRT used to find an unblocked point in find_non_blocked_goal()");
DEFINE_NS_FLOAT64(
    kPlanningParamModule, escape, goal_change_threshold, 0.9,
    "A newly-found unblocked goal must be this much closer to the start position than the "
    "previous point in order to be used (m).");

DEFINE_NS_FLOAT64(
    kPlanningParamModule, pivot, radius_multiplier, 1.5,
    "A newly-found unblocked goal must be this much closer to the start position than the "
    "previous point in order to be used (m).");

// Controls at which ball speed we should try to go directly to the ball
// or to move behind it and in the same direction as it
//
// Low number indicates that it will always try to choose a point for the
// robot behind the velocity vector
//
// High number indicates that it will always try to choose a point nearest
// to the current robot position
DEFINE_NS_FLOAT64(kPlanningParamModule, collect, ball_speed_approach_direction_cutoff, 1.0,
                  "Controls at which ball speed we should try to go directly to the ball "
                  "or to move behind it and in the same direction as it (m/s)");
// How much to scale the accelerations by as a percent of the normal
// acceleration
//
// Approach acceleration controls all the movement from the start location
// to right before touching the ball
DEFINE_NS_FLOAT64(
    kPlanningParamModule, collect, approach_accel_scale, 1.0,
    "How much to scale the accelerations by as a percent of the normal acceleration (unitless)");
// Control acceleration controls the touch to stop
// Lower this if we decelerate too quickly for the dribbler to keep a back
// spin on
DEFINE_NS_FLOAT64(kPlanningParamModule, collect, control_accel_scale, 1.0,
                  "Control acceleration controls the touch to stop (unitless)");
// How far away from the ball to target for the approach
// This should be tuned so that the end of approach places the dribbler
// touching the ball Increase the distance so that when we overshoot, it
// doesn't hit the ball Ideally this is 0
DEFINE_NS_FLOAT64(kPlanningParamModule, collect, approach_dist_target, 0.01,
                  "How far away from the ball to target for the approach (m)");
// At what speed should we be when we touch the ball (Well, the difference
// in speed between the ball and robot)
// Should be as low as possible where
// we still are able to touch the ball and control it If we are slamming
// into the ball decrease this number If we aren't even touching it to the
// dribbler, increase this number (And check the approach_dist_target)
DEFINE_NS_FLOAT64(kPlanningParamModule, collect, touch_delta_speed, 0.01,
                  "At what speed should we be when we touch the ball (m/s)");
DEFINE_NS_FLOAT64(kPlanningParamModule, collect, velocity_control_scale, 1.0,
                  "The amount  (unitless)");
// How close to the ball do we have to be before transferring to the control state.
// This should be almost zero. Increase if the noise on vision causes problems and we never
// transition.
DEFINE_NS_FLOAT64(
    kPlanningParamModule, collect, dist_cutoff_to_control, 0.05,
    "How close to the ball do we have to be before transferring to the control state (m)");
// How close to the ball do we need to be before transferring back to the approach state and
// restarting. This should be just above zero to account for noise in vision.
DEFINE_NS_FLOAT64(
    kPlanningParamModule, collect, dist_cutoff_to_approach, 1,
    "How close to the ball do we need to be before transferring back to the approach state "
    "and restarting (m)");
// How close to the target velocity do we have to be before transferring to the control state.
// This doesn't matter as much right now. It's good to be close, but it's not too big of deal.
// If we are trying to transition with a large delta velocity between the robot and the ball
// decrease this number.
DEFINE_NS_FLOAT64(
    kPlanningParamModule, collect, vel_cutoff_to_control, 0.02,
    "How close to the target velocity do we have to be before transferring to the control "
    "state (m/s)");
// How much extra room should we stay at the delta speed before slowing down.
// This is really a percent of the minimum stop distance to fully stop given
// the current velocity. This should be tuned such that we dont drive too far
// through the ball. A number of 1 should mean a constant acceleration
// through the entire sequence. Increasing this number makes the robot state
// at delta velocity longer.
DEFINE_NS_FLOAT64(
    kPlanningParamModule, collect, stop_dist_scale, 0.8,
    "Portion of maximum stopping distance for which we stay at the delta speed (unitless)");
// Gain on the averaging function to smooth the target point to intercept
// This is due to the high flucations in the ball velocity frame to frame
// a*new_point + (1-a)*old_point
// The lower the number, the less noise affects the system, but the slower
// it responds to changes The higher the number, the more noise affects the
// system, but the faster it responds to changes
DEFINE_NS_FLOAT64(
    kPlanningParamModule, collect, target_point_lowpass_gain, 0.6,
    "Gain on the averaging function to smooth the target point to intercept (unitless)");

// How much of the ball seed to contact the ball with
// before slowing down to dampen the initial hit
DEFINE_NS_FLOAT64(kPlanningParamModule, settle, ball_speed_percent_for_dampen, 0.1,
                  "How much of the ball seed to contact the ball with before slowing down to "
                  "dampen the initial hit (unitless)")
// Closest dist to start searching for intercept points
DEFINE_NS_FLOAT64(kPlanningParamModule, settle, search_start_dist, 0.0,
                  "Closest dist to start searching for intercept points (m)")
// Furthest dist to search for intercept points
DEFINE_NS_FLOAT64(kPlanningParamModule, settle, search_end_dist, 6.0,
                  "Furthest dist to start searching for intercept points (m)")
// What dist increment to search for intercepts
DEFINE_NS_FLOAT64(kPlanningParamModule, settle, search_inc_dist, 0.2,
                  "Search increment for intercept points (m)")
// How much sooner should we reach the intercept point than we need to
// This is a percent of the calculated intercept time
// Numbers greater than 1 mean we increase intercept time needed by X% over
// actual Numbers less than 1 mean we get there X% faster than we plan
// (Shouldn't ever happen)
DEFINE_NS_FLOAT64(
    kPlanningParamModule, settle, intercept_buffer_time, 0.3,
    "How much sooner we should reach the intercept point, compared to the total time (unitless)")
// Gain on the averaging function to smooth the target point to intercept
// This is due to the high flucations in the ball velocity frame to frame
// a*new_point + (1-a)*old_point
// The lower the number, the less noise affects the system, but the slower
// it responds to changes The higher the number, the more noise affects the
// system, but the faster it responds to changes
DEFINE_NS_FLOAT64(kPlanningParamModule, settle, target_point_gain, 0.5,
                  "Gain for LPF for ball target position (unitless)")
// Gain on the averaging function to smooth the ball velocity to for any
// motion commands This is due to the high flucations in the ball velocity
// frame to frame a*new_point + (1-a)*old_point The lower the number, the less
// noise affects the system, but the slower it responds to changes The
// higher the number, the more noise affects the system, but the faster it
// responds to changes
DEFINE_NS_FLOAT64(kPlanningParamModule, settle, ball_vel_gain, 0.9,
                  "Gain for LPF to smooth ball velocity (unitless)")
// Distance between robot and closest point on ball line such that we move
// directly into the ball line instead of trying to find the point we hit
// first This does take into account slow moving balls in which we should
// move onto the ball to capture it
DEFINE_NS_FLOAT64(
    kPlanningParamModule, settle, shortcut_dist, 0.09,
    "Distance between robot and closest point on ball line to move directly to the line (m)")
// If the ball velocity angle changes by a large amount
// we want to quickly react and clear all the smoothing filters
// Lower numbers means it reacts faster, but more chance for false positives
// Higher numbers means slower reaction, but less false positives
DEFINE_NS_FLOAT64(
    kPlanningParamModule, settle, max_ball_angle_for_reset, 20,
    "Distance between robot and closest point on ball line to move directly to the line (deg)")
// If the ball velocity itself changes by a large amount
// we want to quickly react and clear all the smoothing filters
// Lower numbers means it reacts faster, but more chance for false positives
// Higher numbers means slower reaction, but less false positives
DEFINE_NS_FLOAT64(kPlanningParamModule, settle, max_ball_vel_for_path_reset, 2.0,
                  "Change in ball velocity required to reset all filters (m/s)")
// Max angle between ball and target bounce direction
DEFINE_NS_FLOAT64(kPlanningParamModule, settle, max_bounce_angle, 45,
                  "Max angle between ball and target bounce direction (deg)")

}  // namespace planning
