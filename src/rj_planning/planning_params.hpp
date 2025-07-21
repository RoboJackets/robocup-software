#pragma once

#include <rj_param_utils/param.hpp>

namespace planning {

constexpr auto kPlanningParamModule = "planning";

DECLARE_FLOAT64(kPlanningParamModule, timeout);

DECLARE_NS_FLOAT64(kPlanningParamModule, constraints, max_translational_speed);
DECLARE_NS_FLOAT64(kPlanningParamModule, constraints, max_translational_accel);
DECLARE_NS_FLOAT64(kPlanningParamModule, constraints, max_rotational_speed);
DECLARE_NS_FLOAT64(kPlanningParamModule, constraints, max_rotational_accel);

DECLARE_NS_FLOAT64(kPlanningParamModule, replanner, pos_change_threshold);
DECLARE_NS_FLOAT64(kPlanningParamModule, replanner, vel_change_threshold);
DECLARE_NS_FLOAT64(kPlanningParamModule, replanner, partial_replan_lead_time);
DECLARE_NS_FLOAT64(kPlanningParamModule, replanner, off_path_threshold);

DECLARE_NS_BOOL(kPlanningParamModule, rrt, enable_debug_drawing);
DECLARE_NS_FLOAT64(kPlanningParamModule, rrt, step_size);
DECLARE_NS_FLOAT64(kPlanningParamModule, rrt, goal_bias);
DECLARE_NS_FLOAT64(kPlanningParamModule, rrt, waypoint_bias);
DECLARE_NS_INT64(kPlanningParamModule, rrt, min_iterations);
DECLARE_NS_INT64(kPlanningParamModule, rrt, max_iterations);

DECLARE_NS_FLOAT64(kPlanningParamModule, intermediate, min_scale);
DECLARE_NS_FLOAT64(kPlanningParamModule, intermediate, max_scale);
DECLARE_NS_FLOAT64(kPlanningParamModule, intermediate, min_angle);
DECLARE_NS_FLOAT64(kPlanningParamModule, intermediate, max_angle);
DECLARE_NS_INT64(kPlanningParamModule, intermediate, num_intermediates);
DECLARE_NS_FLOAT64(kPlanningParamModule, intermediate, step_size);

DECLARE_NS_FLOAT64(kPlanningParamModule, escape, step_size);
DECLARE_NS_FLOAT64(kPlanningParamModule, escape, goal_change_threshold);

DECLARE_NS_FLOAT64(kPlanningParamModule, pivot, radius_multiplier);

DECLARE_NS_FLOAT64(kPlanningParamModule, collect, ball_speed_approach_direction_cutoff);
DECLARE_NS_FLOAT64(kPlanningParamModule, collect, approach_accel_scale);
DECLARE_NS_FLOAT64(kPlanningParamModule, collect, control_accel_scale);
DECLARE_NS_FLOAT64(kPlanningParamModule, collect, approach_dist_target);
DECLARE_NS_FLOAT64(kPlanningParamModule, collect, touch_delta_speed);
DECLARE_NS_FLOAT64(kPlanningParamModule, collect, velocity_control_scale);
DECLARE_NS_FLOAT64(kPlanningParamModule, collect, dist_cutoff_to_control);
DECLARE_NS_FLOAT64(kPlanningParamModule, collect, dist_cutoff_to_approach);
DECLARE_NS_FLOAT64(kPlanningParamModule, collect, vel_cutoff_to_control);
DECLARE_NS_FLOAT64(kPlanningParamModule, collect, stop_dist_scale);
DECLARE_NS_FLOAT64(kPlanningParamModule, collect, target_point_lowpass_gain);

DECLARE_NS_FLOAT64(kPlanningParamModule, settle, ball_speed_percent_for_dampen);
DECLARE_NS_FLOAT64(kPlanningParamModule, settle, search_start_dist);
DECLARE_NS_FLOAT64(kPlanningParamModule, settle, search_start_dist);
DECLARE_NS_FLOAT64(kPlanningParamModule, settle, search_inc_dist);
DECLARE_NS_FLOAT64(kPlanningParamModule, settle, search_end_dist);
DECLARE_NS_FLOAT64(kPlanningParamModule, settle, intercept_buffer_time);
DECLARE_NS_FLOAT64(kPlanningParamModule, settle, target_point_gain);
DECLARE_NS_FLOAT64(kPlanningParamModule, settle, ball_vel_gain);
DECLARE_NS_FLOAT64(kPlanningParamModule, settle, shortcut_dist);
DECLARE_NS_FLOAT64(kPlanningParamModule, settle, max_ball_angle_for_reset);
DECLARE_NS_FLOAT64(kPlanningParamModule, settle, max_ball_vel_for_path_reset);
DECLARE_NS_FLOAT64(kPlanningParamModule, settle, max_bounce_angle);

}  // namespace planning
