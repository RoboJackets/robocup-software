#pragma once

#include <Geometry2d/Point.hpp>
#include <Configuration.hpp>
#include <boost/optional.hpp>

/**
 * This class contains the motion constraints that the high-level logic sets for
 * a robot.
 * For position: set EITHER @motionTarget OR @targetWorldVel.
 * For angle: set EITHER @targetAngleVel OR @faceTarget.
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
    float maxSpeed;
    float maxAcceleration;


    /// Default constraint values supplied by config
    ////////////////////////////////////////////////////////////////////////////////

    static void createConfiguration(Configuration* cfg);
    static ConfigDouble* _max_acceleration;
    static ConfigDouble* _max_speed;
    static ConfigDouble* _max_rotation_speed;
    static ConfigDouble* _max_rotation_acceleration;
    static ConfigDouble* _replan_threshold;
    static ConfigDouble* _replan_lead_time;
};
