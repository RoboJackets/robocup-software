#pragma once

#include <Geometry2d/Point.hpp>

namespace Planning {

    /**
     * This class represents a robot's motion "state" at a given time, including
     * position and velocity.
     */
    struct MotionInstant {
        explicit MotionInstant(Geometry2d::Point pos = {0, 0},
                               Geometry2d::Point vel = {0, 0})
            : pos(pos), vel(vel) {}

        Geometry2d::Point pos;
        Geometry2d::Point vel;

    //    friend bool operator==(const MotionInstant& a, const MotionInstant& b) {
    //        return a.pos == b.pos && a.vel == b.vel;
    //    }

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
        explicit AngleInstant(float angle = 0, float angleVel = 0) : angle(angle), angleVel(angleVel) {};
        float angle;
        float angleVel;

        //TODO ashaw596 implement stream operator
    };


    /**
     * This class represents a robot's motion and rotation "state" at a given time, including
     * position, velocity, angle, and rotationVelocity.
     */
    struct RobotInstant {
        explicit RobotInstant(MotionInstant motion = MotionInstant(),
                              AngleInstant angle = AngleInstant())
                : motion(motion), angle(angle) {}
        MotionInstant motion;
        AngleInstant angle;

        //TODO ashaw596  implement stream operator
    };
}  // namespace Planning
