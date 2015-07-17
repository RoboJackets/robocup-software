#pragma once

#include <Geometry2d/Point.hpp>
#include <boost/optional.hpp>
#include "planning/MotionInstant.hpp"


namespace Planning {
/**
 * This class contains the motion constraints that the high-level logic sets for a robot.
 * For position: set EITHER @motionTarget OR @targetWorldVel.
 * For angle: set EITHER @targetAngleVel OR @faceTarget.
 */
    class MotionCommand {

    public:
        MotionCommand() : usePath(false), _targetMotionInstant(), _targetWorldVel() {};

        void setPathTarget(MotionInstant target) {
            _targetMotionInstant = target;
            usePath = true;
        }

        void setWorldVel(Geometry2d::Point targetVel) {
            _targetWorldVel = targetVel;
            usePath = false;
        }

        MotionInstant getPlanningTarget() const {
            return _targetMotionInstant;
        }

        Geometry2d::Point getWorldVel() const {
            return _targetWorldVel;
        }

        bool usePathPlanning() const {
            return usePath;
        }

    private:
        /// A point on the field that the robot should use path-planning to get to
        MotionInstant _targetMotionInstant;

        /// Set the velocity in world coordinates directly (circumvents path planning)
        Geometry2d::Point _targetWorldVel;

        //True when _targetPos should be used. False when worldVel override should be used
        bool usePath;
    };
}