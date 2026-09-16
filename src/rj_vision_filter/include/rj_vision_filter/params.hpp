#pragma once

#include <cstdint>

namespace vision_filter {

/**
 * @brief All tunable parameters for the vision filter, read from ROS2
 * parameters once at startup (and kept live-updated) by VisionFilter, and
 * passed by const reference down to the plain (non-Node) classes that use
 * them.
 */
struct VisionFilterParams {
    double vision_loop_dt = 1.0 / 60.0;
    int64_t max_num_cameras = 12;
    double publish_hz = 60.0;

    struct {
        double init_covariance = 100.0;
        double process_noise = 0.1;
        double observation_noise = 2.0;
    } ball;

    struct {
        double init_covariance = 100.0;
        double process_noise = 0.5;
        double observation_noise = 2.0;
        double orientation_scale = 1.0;
    } robot;

    struct {
        double mhkf_radius_cutoff = 0.5;
        bool use_mhkf = true;
        int64_t max_num_kalman_balls = 10;
        int64_t max_num_kalman_robots = 10;
    } camera;

    struct {
        double max_time_outside_vision = 0.2;
    } kalman_ball;

    struct {
        double max_time_outside_vision = 0.5;
    } kalman_robot;

    struct {
        int64_t init = 2;
        int64_t inc = 2;
        int64_t dec = 1;
        int64_t max = 20;
        int64_t min = 1;
    } filter_health;

    struct {
        int64_t slow_kick_hist_length = 5;
        int64_t fast_kick_hist_length = 3;
        double fast_kick_timeout = 1.0;
        double slow_kick_timeout = 0.5;
        double same_kick_timeout = 0.5;
        double fast_acceleration_trigger = 750.0;
        // TODO: Unused (dead since before the rj_param_utils removal). Safe to remove.
        double slow_robot_dist_filter_cutoff = 3.0;
        double slow_one_robot_within_dist = 0.15;
        double slow_any_robot_past_dist = 0.16;
        double slow_min_ball_speed = 0.6;
        double slow_max_kick_angle = 0.34;
    } kick_detector;

    struct {
        double robot_body_lin_dampen = 0.9;
        double robot_mouth_lin_dampen = 0.3;
        double robot_body_angle_dampen = 0.0;
        double robot_mouth_angle_dampen = 0.0;
    } bounce;

    struct {
        double ball_merger_power = 1.5;
    } world_ball;

    struct {
        double robot_merger_power = 1.5;
    } world_robot;
};

}  // namespace vision_filter
