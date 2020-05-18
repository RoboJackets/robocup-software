#pragma once

#include <variant>

#include <Geometry2d/Point.hpp>
#include <Geometry2d/Pose.hpp>
#include <WorldState.hpp>
#include "Utils.hpp"
#include "planning/MotionInstant.hpp"
#include "planning/trajectory/Trajectory.hpp"

namespace Planning {
// todo(Ethan) discuss whether we want to have settle with bounce
struct SettleCommand {
    std::optional<Geometry2d::Point> target;
};
struct CollectCommand {};
struct LineKickCommand {
    Geometry2d::Point target;
};

/**
 * An empty "do-nothing" motion command.
 */
struct EmptyCommand {};

/**
 * Move to a particular target with a particular velocity, avoiding obstacles.
 */
// note: as of now the heading and angular velocity are ignored
// TODO use heading and angular velocity or change this to a MotionInstant
struct PathTargetCommand {
    RobotInstant pathGoal;
    std::optional<double> angle_override;
};

/**
 * Move with a particular velocity.
 */
struct WorldVelCommand {
    Geometry2d::Point worldVel;
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
 * Intercept a moving ball, disregarding whether or not we can actually capture
 * it.
 *
 * Designed for goal defense.
 */
struct InterceptCommand {
    Geometry2d::Point target;
};

using MotionCommand =
    std::variant<EmptyCommand, PathTargetCommand, WorldVelCommand, PivotCommand,
                 SettleCommand, CollectCommand, LineKickCommand,
                 InterceptCommand>;

}  // namespace Planning
