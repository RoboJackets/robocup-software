#pragma once

#include <geometry2d/point.h>
#include <geometry2d/pose.h>

#include <optional>

namespace Planning {

/**
 * This class represents a robot's motion "state" at a given time, including
 * position and velocity.
 */
struct MotionInstant {
    explicit MotionInstant(geometry2d::Point pos = {0, 0},
                           geometry2d::Point vel = {0, 0})
        : pos(pos), vel(vel) {}

    geometry2d::Point pos;
    geometry2d::Point vel;

    friend std::ostream& operator<<(std::ostream& stream,
                                    const MotionInstant& instant) {
        return stream << "MotionInstant(pos=" << instant.pos
                      << ", vel=" << instant.vel << ")";
    }
};

/**
 * This class represents a robot's angle "state" at a given time, including
 * angle and rotation velocity.
 */
struct AngleInstant {
    explicit AngleInstant(std::optional<float> angle = std::nullopt,
                          std::optional<float> angleVel = std::nullopt)
        : angle(angle), angleVel(angleVel){};
    std::optional<float> angle;
    std::optional<float> angleVel;

    friend std::ostream& operator<<(std::ostream& stream,
                                    const AngleInstant& angle_instant) {
        const auto angle_str =
            angle_instant.angle ? std::to_string(*angle_instant.angle) : "None";
        const auto angle_vel_str = angle_instant.angleVel
                                       ? std::to_string(*angle_instant.angleVel)
                                       : "None";
        stream << "AngleInstant(" << angle_str << ", " << angle_vel_str << ")";
        return stream;
    }
};

/**
 * This class represents a robot's motion and rotation "state" at a given time,
 * including
 * position, velocity, angle, and rotationVelocity.
 */
struct RobotInstant {
    explicit RobotInstant(MotionInstant motion = MotionInstant(),
                          std::optional<AngleInstant> angle = std::nullopt)
        : motion(motion), angle(angle) {}
    MotionInstant motion;
    std::optional<AngleInstant> angle;

    geometry2d::Pose pose() {
        if (angle && angle->angle) {
            return geometry2d::Pose(motion.pos, angle->angle.value());
        } else {
            return geometry2d::Pose(motion.pos, 0);
        }
    }

    geometry2d::Twist twist() {
        if (angle && angle->angleVel) {
            return geometry2d::Twist(motion.vel, angle->angleVel.value());
        } else {
            return geometry2d::Twist(motion.vel, 0);
        }
    }

    // TODO ashaw596  implement stream operator
};
}  // namespace Planning
