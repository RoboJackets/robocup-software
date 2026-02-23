#pragma once

#include <cstdint>

namespace vision_filter {

/**
 * @brief Configuration parameters for the vision filter system.
 *
 * All values are loaded from the YAML parameter files
 * (sim_params.yaml or real_params.yaml) via the ROS2 parameter server.
 * The VisionFilter node populates this struct and passes it to
 * non-node classes.
 */
struct VisionFilterConfig {
    // General
    double vision_loop_dt = 1.0 / 60.0;
    int64_t max_num_cameras = 12;
    double publish_hz = 60.0;

    // Filter health
    struct {
        int64_t init = 2;
        int64_t inc = 2;
        int64_t dec = 1;
        int64_t max = 20;
        int64_t min = 1;
    } filter_health;

    // Ball Kalman filter
    struct {
        double init_covariance = 100.0;
        double process_noise = 0.1;
        double observation_noise = 2.0;
    } ball;

    // Robot Kalman filter
    struct {
        double init_covariance = 100.0;
        double process_noise = 0.5;
        double observation_noise = 2.0;
        double orientation_scale = 1.0;
    } robot;

    // Camera
    struct {
        double mhkf_radius_cutoff = 0.5;
        bool use_mhkf = true;
        int64_t max_num_kalman_balls = 10;
        int64_t max_num_kalman_robots = 10;
    } camera;

    // Ball bounce
    struct {
        double robot_body_lin_dampen = 0.9;
        double robot_mouth_lin_dampen = 0.3;
        double robot_body_angle_dampen = 0.0;
        double robot_mouth_angle_dampen = 0.0;
    } bounce;

    // World ball
    struct {
        double ball_merger_power = 1.5;
    } world_ball;

    // World robot
    struct {
        double robot_merger_power = 1.5;
    } world_robot;

    // Kalman ball
    struct {
        double max_time_outside_vision = 0.2;
    } kalman_ball;

    // Kalman robot
    struct {
        double max_time_outside_vision = 0.5;
    } kalman_robot;

    // Kick detector
    struct {
        double fast_acceleration_trigger = 750.0;
        int64_t fast_kick_hist_length = 3;
        double fast_kick_timeout = 1.0;
        double same_kick_timeout = 0.5;
        double slow_any_robot_past_dist = 0.16;
        int64_t slow_kick_hist_length = 5;
        double slow_kick_timeout = 0.5;
        double slow_max_kick_angle = 0.34;
        double slow_min_ball_speed = 0.6;
        double slow_one_robot_within_dist = 0.15;
        double slow_robot_dist_filter_cutoff = 3.0;
    } kick_detector;
};

}  // namespace vision_filter
