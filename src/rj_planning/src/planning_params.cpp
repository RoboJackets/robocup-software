#include "rj_planning/planning_params.hpp"

#include <rj_common/ros_params.hpp>

namespace planning {

PlanningParams load_planning_params(rclcpp::Node& node) {
    PlanningParams p{};
    p.timeout = declare_required_param<double>(node, "timeout",
                                               "Timeout for vision data to reach planning (s)");
    p.constraints.max_translational_speed =
        declare_required_param<double>(node, "constraints.max_translational_speed",
                                       "Maximum translational speed for planning (m/s)");
    p.constraints.max_translational_accel =
        declare_required_param<double>(node, "constraints.max_translational_accel",
                                       "Maximum translational acceleration for planning (m/s^2)");
    p.constraints.max_rotational_speed = declare_required_param<double>(
        node, "constraints.max_rotational_speed", "Maximum rotational speed for planning (rad/s)");
    p.constraints.max_rotational_accel =
        declare_required_param<double>(node, "constraints.max_rotational_accel",
                                       "Maximum rotational acceleration for planning (rad/s^2)");
    p.replanner.pos_change_threshold =
        declare_required_param<double>(node, "replanner.pos_change_threshold",
                                       "Goal position change (m) below which no replanning occurs");
    p.replanner.vel_change_threshold = declare_required_param<double>(
        node, "replanner.vel_change_threshold",
        "Goal velocity change (m/s) below which no replanning occurs");
    p.replanner.partial_replan_lead_time = declare_required_param<double>(
        node, "replanner.partial_replan_lead_time",
        "Portion of the path (s) kept unchanged in a partial replan");
    p.replanner.off_path_threshold = declare_required_param<double>(
        node, "replanner.off_path_threshold",
        "Distance (m) from the planned trajectory that forces a replan");
    p.intermediate.min_scale = declare_required_param<double>(
        node, "intermediate.min_scale", "Minimum length for intermediate point (m)");
    p.intermediate.max_scale = declare_required_param<double>(
        node, "intermediate.max_scale", "Maximum length for intermediate point (m)");
    p.intermediate.min_angle = declare_required_param<double>(
        node, "intermediate.min_angle", "Minimum angle for intermediate point (deg)");
    p.intermediate.max_angle = declare_required_param<double>(
        node, "intermediate.max_angle", "Maximum angle for intermediate point (deg)");
    p.intermediate.num_intermediates = declare_required_param<int>(
        node, "intermediate.num_intermediates", "Number of intermediate points tried");
    p.intermediate.step_size = declare_required_param<double>(
        node, "intermediate.step_size", "Step size for testing intermediates (m)");
    p.pivot.radius_multiplier = declare_required_param<double>(node, "pivot.radius_multiplier",
                                                               "Scale applied to the pivot radius");
    p.collect.ball_speed_approach_direction_cutoff = declare_required_param<double>(
        node, "collect.ball_speed_approach_direction_cutoff",
        "Ball speed (m/s) below which collect drives straight at the ball instead of behind it");
    p.collect.approach_accel_scale = declare_required_param<double>(
        node, "collect.approach_accel_scale", "Scale on max acceleration during the fine approach");
    p.collect.approach_dist_target = declare_required_param<double>(
        node, "collect.approach_dist_target",
        "Distance from the ball (m) targeted by the coarse approach");
    p.collect.touch_delta_speed = declare_required_param<double>(
        node, "collect.touch_delta_speed", "Speed relative to the ball (m/s) when touching it");
    p.collect.dist_cutoff_to_control = declare_required_param<double>(
        node, "collect.dist_cutoff_to_control",
        "Distance to the ball (m) at which collect switches to control");
    p.settle.ball_speed_percent_for_dampen = declare_required_param<double>(
        node, "settle.ball_speed_percent_for_dampen",
        "Fraction of ball speed to meet the ball with before dampening");
    p.settle.search_start_dist = declare_required_param<double>(
        node, "settle.search_start_dist", "Closest distance to search for intercept points (m)");
    p.settle.search_end_dist = declare_required_param<double>(
        node, "settle.search_end_dist", "Furthest distance to search for intercept points (m)");
    p.settle.search_inc_dist = declare_required_param<double>(
        node, "settle.search_inc_dist", "Search increment for intercept points (m)");
    p.settle.intercept_buffer_time = declare_required_param<double>(
        node, "settle.intercept_buffer_time",
        "How much sooner to reach the intercept point, as a fraction of the intercept time");
    p.settle.target_point_gain = declare_required_param<double>(
        node, "settle.target_point_gain", "Low-pass filter gain for the intercept target");
    p.settle.ball_vel_gain = declare_required_param<double>(
        node, "settle.ball_vel_gain", "Low-pass filter gain for ball velocity");
    p.settle.shortcut_dist = declare_required_param<double>(
        node, "settle.shortcut_dist",
        "Distance to the ball line (m) within which settle moves straight onto it");
    p.settle.max_ball_angle_for_reset =
        declare_required_param<double>(node, "settle.max_ball_angle_for_reset",
                                       "Change in ball direction (deg) that resets the filters");
    p.settle.max_ball_vel_for_path_reset =
        declare_required_param<double>(node, "settle.max_ball_vel_for_path_reset",
                                       "Change in ball velocity (m/s) that resets the filters");
    p.settle.max_bounce_angle =
        declare_required_param<double>(node, "settle.max_bounce_angle",
                                       "Max angle between ball and target bounce direction (deg)");
    return p;
}

}  // namespace planning
