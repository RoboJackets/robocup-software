#pragma once

#include <configuration.hpp>
#include <rj_geometry/point.hpp>
#include <configuration.hpp>

/**
 * This class contains the motion constraints that the high-level logic sets for
 * a robot.
 * For position: set EITHER @motion_target OR @target_world_vel.
 * For angle: set EITHER @target_angle_vel OR @face_target.
 */
struct MotionConstraints {
    MotionConstraints();

    /**
     * Each instance has a set of speed/acceleration limits that are used for
     * path following
     * They default to the global config values defined below, but can be
     * overridden by setting these.
     * This is useful for going slower while carrying the ball or when trying to
     * do precise movements.
     */
    double max_speed;
    double max_acceleration;

    /// Default constraint values supplied by config
    ////////////////////////////////////////////////////////////////////////////////

    static double default_max_speed() { return *max_speed_config; }

    static void create_configuration(Configuration* cfg);
    static ConfigDouble* max_acceleration_config;
    static ConfigDouble* max_speed_config;
};
