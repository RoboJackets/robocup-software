#pragma once

#include <Geometry2d/Point.hpp>
#include <boost/optional.hpp>
#include "planning/MotionInstant.hpp"

namespace Planning {


class MotionCommand {
public:
    enum CommandType { PathTarget, WorldVel };

    MotionCommand()
        : _commandType(WorldVel),
          _targetMotionInstant(),
          _targetWorldVel() 
          {}

    void setPathTarget(MotionInstant target) {
        _targetMotionInstant = target;
        _commandType = PathTarget;
    }

    void setWorldVel(Geometry2d::Point targetVel) {
        _targetWorldVel = targetVel;
        _commandType = WorldVel;
    }

    MotionInstant getPlanningTarget() const { return _targetMotionInstant; }

    Geometry2d::Point getWorldVel() const { return _targetWorldVel; }

    CommandType getCommandType() const { return _commandType; }

private:
    /// A point on the field that the robot should use path-planning to get to
    MotionInstant _targetMotionInstant;

    /// Set the velocity in world coordinates directly (circumvents path
    /// planning)
    Geometry2d::Point _targetWorldVel;

    // The type of command
    CommandType _commandType;
};
}