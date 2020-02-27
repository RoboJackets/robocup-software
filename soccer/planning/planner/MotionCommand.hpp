#pragma once

#include <variant>

#include <Geometry2d/Point.hpp>
#include <Geometry2d/Pose.hpp>
#include <WorldState.hpp>
#include "planning/trajectory/Trajectory.hpp"
#include "planning/MotionInstant.hpp"
#include "Utils.hpp"

namespace Planning {

//todo(Ethan) delete these
struct SettleCommand{};
struct CollectCommand{};
struct LineKickCommand{
    Geometry2d::Point target;
};

/**
 * An empty "do-nothing" motion command.
 */
struct EmptyCommand {};

/**
 * Move to a particular target with a particular velocity, avoiding obstacles.
 */
struct PathTargetCommand {
    RobotInstant pathGoal;
};

/**
 * Move with a particular velocity.
 */
struct WorldVelTargetCommand {
    Geometry2d::Twist worldVel;
};

/**
 * Pivot around a given point, with a given target angle.
 *
 * The robot will face the pivotPoint throughout the command.
 */
struct PivotCommand {
    Geometry2d::Point pivotPoint;
    Geometry2d::Point pivotTarget;
};

/**
 * Move to a particular point, ignoring obstacles.
 *
 * Designed to be used for tuning.
 */
struct TuningPathCommand {
    MotionInstant pathGoal;
};

/**
 * Intercept a moving ball, disregarding whether or not we can actually capture it.
 *
 * Designed for goal defense.
 */
struct InterceptCommand {
    Geometry2d::Point target;
};

using MotionCommand = std::variant<
        EmptyCommand,
        PathTargetCommand,
        WorldVelTargetCommand,
        PivotCommand,
        TuningPathCommand,
        SettleCommand,
        CollectCommand,
        LineKickCommand,
        InterceptCommand
        >;

}  // namespace Planning
