#pragma once

#include <planning/RotationConstraints.hpp>

#include "PathSmoothing.hpp"
#include "planning/Instant.hpp"
#include "planning/Trajectory.hpp"

namespace Planning {

/**
 * @brief Find the final (tangential) speed, at most velocity_final, that we can
 * achieve moving along the path.
 *
 * @param velocity_initial The initial speed along the path at the beginning of
 * a segment
 * @param velocity_final The maximum final speed along the path at the end of
 * the segment, from other constraints
 * @param displacement The total displacement of the segment
 * @param max_accel The maximum (tangential) acceleration over this segment
 */
double limit_acceleration(double velocity_initial, double velocity_final,
                          double displacement, double max_accel);

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
Trajectory profile_velocity(const BezierPath& path, double initial_speed,
                           double final_speed,
                           const MotionConstraints& constraints,
                           RJ::Time initial_time = RJ::now());

}  // namespace Planning
