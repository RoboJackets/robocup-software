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

/*
 * Capture a ball.
 * - targetFacePoint defines a target angle in terms of a point
 *   that we want to aim towards during the final instant of the trajectory
 *   (designed to be used for a Line Kick).
 * - targetSpeed only applies when approaching directly toward a slow moving
 *   ball. Otherwise target speed is defined relative to the ball velocity
 *   (by CapturePlanner::_touchDeltaSpeed and _ballSpeedPercentForDampen)
 */
struct CaptureCommand {
    std::optional<Geometry2d::Point> targetFacePoint;
    std::optional<double> targetSpeed;
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
        CaptureCommand,
        InterceptCommand,
        SettleCommand,
        CollectCommand
        >;

}  // namespace Planning
