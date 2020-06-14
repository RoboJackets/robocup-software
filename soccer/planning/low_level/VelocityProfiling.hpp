#pragma once

#include <planning/RotationConstraints.hpp>

#include "PathSmoothing.hpp"
#include "planning/Instant.hpp"
#include "planning/Trajectory.hpp"

namespace Planning {

/**
 * @brief Create a trajectory with the given path by calculating the maximum
 * possible velocity at each points given constraints on speed and acceleration.
 *
 * @param path A spatial path along which to move.
 * @param initial_speed The initial tangential speed along the path at
 * `initial_time`.
 * @param final_speed The (goal) final tangential speed along the path.
 * @param initial_time The start time of the trajectory (usually close to now,
 * or the most recent instant from vision).
 * @return A profiled time-indexed trajectory that follows velocity constraints.
 */
Trajectory ProfileVelocity(const BezierPath& path, double initial_speed,
                           double final_speed,
                           const MotionConstraints& constraints,
                           RJ::Time initial_time = RJ::now());

}  // namespace Planning
