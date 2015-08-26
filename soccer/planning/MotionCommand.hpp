#pragma once

#include <Geometry2d/Point.hpp>
#include <boost/optional.hpp>
#include "planning/MotionInstant.hpp"

namespace Planning {
/**
 * This class contains the motion constraints that the high-level logic sets for
 * a robot.
 * For position: set EITHER @motionTarget OR @targetWorldVel.
 * For angle: set EITHER @targetAngleVel OR @faceTarget.
 */
class MotionCommand {
public:
    enum CommandType { PathTarget, WorldVel, DirectTarget };

    MotionCommand()
        : _commandType(WorldVel),
          _targetMotionInstant(),
          _targetWorldVel(),
          _directTarget(),
          _directEndSpeed(){};

    void setPathTarget(MotionInstant target) {
        _targetMotionInstant = target;
        _commandType = PathTarget;
    }

    void setWorldVel(Geometry2d::Point targetVel) {
        _targetWorldVel = targetVel;
        _commandType = WorldVel;
    }

    void setDirectTarget(Geometry2d::Point target, float endSpeed) {
        _directTarget = target;
        _directEndSpeed = endSpeed;
        _commandType = DirectTarget;
    }

    MotionInstant getPlanningTarget() const { return _targetMotionInstant; }

    float getDirectTarget(Geometry2d::Point& directTarget) const {
        directTarget = _directTarget;
        return _directEndSpeed;
    }

    Geometry2d::Point getWorldVel() const { return _targetWorldVel; }

    CommandType getCommandType() const { return _commandType; }

private:
    /// A point on the field that the robot should use path-planning to get to
    MotionInstant _targetMotionInstant;

    Geometry2d::Point _directTarget;
    float _directEndSpeed;

    /// Set the velocity in world coordinates directly (circumvents path
    /// planning)
    Geometry2d::Point _targetWorldVel;

    // The type of command
    CommandType _commandType;
};
}