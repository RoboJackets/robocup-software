#pragma once

#include <planning/RotationConstraints.hpp>

#include "PathSmoothing.hpp"
#include "planning/Instant.hpp"
#include "planning/Trajectory.hpp"

namespace Planning {

/**
 * Create a trajectory with the given path by calculating the maximum possible
 * velocity at each points given constraints on speed and acceleration.
 *
 * @param path
 * @return
 */
Trajectory ProfileVelocity(const BezierPath& path, double initial_speed,
                           double final_speed,
                           const MotionConstraints& constraints,
                           RJ::Time initial_time = RJ::now());

}  // namespace Planning
