#pragma once

#include <Geometry2d/Point.hpp>
#include <planning/Path.hpp>
#include <boost/optional.hpp>


/**
 * This class contains the motion constraints that the high-level logic sets for a robot.
 * For position: set EITHER @motionTarget OR @targetWorldVel.
 * For angle: set EITHER @targetAngleVel OR @faceTarget.
 */
struct MotionConstraints {

    MotionConstraints();

    /**
     * Position
     */

    /// A point on the field that the robot should use path-planning to get to
    boost::optional<Geometry2d::Point> targetPos;
    
    /// Set the velocity in world coordinates directly (circumvents path planning)
    boost::optional<Geometry2d::Point> targetWorldVel;


    /**
     * Angle
     */
    
    /// Angular velocity in rad/s counterclockwise
    boost::optional<float> targetAngleVel;

    /// A global point on the field that the robot should face towards
    boost::optional<Geometry2d::Point> faceTarget;

    /// The speed we should be going when we reach the end of the path
    float endSpeed = 0;

    /**
     * Each instance has a set of speed/acceleration limits that are used for path following
     * They default to the global config values defined below, but can be overridden by setting these.
     * This is useful for going slower while carrying the ball or when trying to do precise movements.
     */
    float maxSpeed;
    float maxAcceleration;

    static void createConfiguration(Configuration *cfg);
    static ConfigDouble *_max_acceleration;
    static ConfigDouble *_max_speed;
};
