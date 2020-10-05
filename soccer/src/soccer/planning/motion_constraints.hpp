#pragma once

/**
 * This class contains the motion constraints that the high-level logic sets for a robot.
 */
struct MotionConstraints {
    static constexpr double kDefaultMaxSpeed = 1.0;
    static constexpr double kDefaultMaxAcceleration = 3.0;

    double max_speed = kDefaultMaxSpeed;
    double max_acceleration = kDefaultMaxAcceleration;
};
