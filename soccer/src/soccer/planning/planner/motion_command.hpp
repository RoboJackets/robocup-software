#pragma once

#include <rj_geometry/point.hpp>
#include <rj_geometry/pose.hpp>
#include <world_state.hpp>
#include <variant>

#include "planning/instant.hpp"
#include "planning/trajectory.hpp"

namespace Planning {
// todo(Ethan) discuss whether we want to have settle with bounce
struct SettleCommand {
    std::optional<rj_geometry::Point> target;
};
struct CollectCommand {};
struct LineKickCommand {
    rj_geometry::Point target;
};

/**
 * An empty "do-nothing" motion command.
 */
struct EmptyCommand {};

struct TargetFaceTangent {};
struct TargetFaceAngle {
    double target;
};
struct TargetFacePoint {
    rj_geometry::Point face_point;
};

using AngleOverride =
    std::variant<TargetFaceTangent, TargetFaceAngle, TargetFacePoint>;
/**
 * Move to a particular target with a particular velocity, avoiding obstacles.
 */
struct PathTargetCommand {
    LinearMotionInstant goal;
    AngleOverride angle_override = TargetFaceTangent{};
};

/**
 * Move with a particular velocity.
 */
struct WorldVelCommand {
    rj_geometry::Point world_vel;
};

/**
 * Pivot around a given point, with a given target angle.
 *
 * The robot will face the pivot_point throughout the command.
 */
struct PivotCommand {
    rj_geometry::Point pivot_point;
    rj_geometry::Point pivot_target;
};

/**
 * Intercept a moving ball, disregarding whether or not we can actually capture
 * it.
 *
 * Designed for goal defense.
 */
struct InterceptCommand {
    rj_geometry::Point target;
};

using MotionCommand =
    std::variant<EmptyCommand, PathTargetCommand, WorldVelCommand, PivotCommand,
                 SettleCommand, CollectCommand, LineKickCommand,
                 InterceptCommand>;

}  // namespace Planning
