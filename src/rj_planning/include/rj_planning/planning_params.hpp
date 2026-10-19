#pragma once

#include <vector>

#include <rclcpp/node.hpp>

#include "rj_planning/primitives/intermediate_params.hpp"

namespace planning {

/// Robot speed and acceleration limits used for planning.
struct ConstraintsParams {
    double max_translational_speed;  // m/s
    double max_translational_accel;  // m/s^2
    double max_rotational_speed;     // rad/s
    double max_rotational_accel;     // rad/s^2
};

/// When the replanner reuses, partially replans, or fully replans a trajectory.
struct ReplannerParams {
    double pos_change_threshold;      // m
    double vel_change_threshold;      // m/s
    double partial_replan_lead_time;  // s
    double off_path_threshold;        // m
};

struct PivotParams {
    double radius_multiplier;
};

struct CollectParams {
    double ball_speed_approach_direction_cutoff;  // m/s
    double approach_accel_scale;
    double approach_dist_target;    // m
    double touch_delta_speed;       // m/s
    double dist_cutoff_to_control;  // m
};

struct SettleParams {
    double ball_speed_percent_for_dampen;
    double search_start_dist;  // m
    double search_end_dist;    // m
    double search_inc_dist;    // m
    double intercept_buffer_time;
    double target_point_gain;
    double ball_vel_gain;
    double shortcut_dist;                // m
    double max_ball_angle_for_reset;     // deg
    double max_ball_vel_for_path_reset;  // m/s
    double max_bounce_angle;             // deg
};

/**
 * @brief Every parameter the planner node reads from /planner.
 *
 * PlannerNode shares these with each planner through PlanRequest::params.
 */
struct PlanningParams {
    double timeout;  // s
    ConstraintsParams constraints;
    ReplannerParams replanner;
    IntermediateParams intermediate;
    PivotParams pivot;
    CollectParams collect;
    SettleParams settle;
};

/**
 * @brief Declares every /planner parameter on `node` (all required, no
 * defaults) and returns their values.
 */
PlanningParams load_planning_params(rclcpp::Node& node);
void apply_planning_params(PlanningParams& planning_params,
                           const std::vector<rclcpp::Parameter>& params);

}  // namespace planning
