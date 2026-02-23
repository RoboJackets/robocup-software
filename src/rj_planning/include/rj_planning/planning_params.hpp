#pragma once
#include <cstdint>

namespace planning {

struct PlanningConfig {
    double timeout = 0.1;

    struct {
        double max_translational_speed = 2.0;
        double max_translational_accel = 2.0;
        double max_rotational_speed = 5.0;
        double max_rotational_accel = 5.0;
    } constraints;

    struct {
        double pos_change_threshold = 0.025;
        double vel_change_threshold = 0.025;
        double partial_replan_lead_time = 0.1;
        double off_path_threshold = 0.1;
    } replanner;

    struct {
        bool enable_debug_drawing = false;
        double step_size = 0.15;
        double goal_bias = 0.3;
        double waypoint_bias = 0.5;
        int64_t min_iterations = 50;
        int64_t max_iterations = 500;
    } rrt;

    struct {
        double min_scale = 0.5;
        double max_scale = 1.5;
        double min_angle = 20;
        double max_angle = 140;
        int64_t num_intermediates = 5;
        double step_size = 0.1;
    } intermediate;

    struct {
        double step_size = 0.1;
        double goal_change_threshold = 0.9;
    } escape;

    struct {
        double radius_multiplier = 1.5;
    } pivot;

    struct {
        double ball_speed_approach_direction_cutoff = 1.0;
        double approach_accel_scale = 1.0;
        double control_accel_scale = 1.0;
        double approach_dist_target = 0.01;
        double touch_delta_speed = 0.01;
        double velocity_control_scale = 1.0;
        double dist_cutoff_to_control = 0.05;
        double dist_cutoff_to_approach = 1.0;
        double vel_cutoff_to_control = 0.02;
        double stop_dist_scale = 0.8;
        double target_point_lowpass_gain = 0.6;
    } collect;

    struct {
        double ball_speed_percent_for_dampen = 0.1;
        double search_start_dist = 0.0;
        double search_end_dist = 6.0;
        double search_inc_dist = 0.2;
        double intercept_buffer_time = 0.3;
        double target_point_gain = 0.5;
        double ball_vel_gain = 0.9;
        double shortcut_dist = 0.09;
        double max_ball_angle_for_reset = 20.0;
        double max_ball_vel_for_path_reset = 2.0;
        double max_bounce_angle = 45.0;
    } settle;
};

}  // namespace planning
