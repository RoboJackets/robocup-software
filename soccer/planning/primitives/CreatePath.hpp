#pragma once

#include "planning/MotionConstraints.hpp"
#include "planning/Trajectory.hpp"
#include "planning/primitives/PathSmoothing.hpp"

namespace Planning::CreatePath {

/**
 * Generate a smooth path from start to goal avoiding obstacles.
 */
Trajectory rrt(const LinearMotionInstant& start,
               const LinearMotionInstant& goal,
               const MotionConstraints& motionConstraints, RJ::Time startTime,
               const Geometry2d::ShapeSet& static_obstacles,
               const std::vector<DynamicObstacle>& dynamic_obstacles = {},
               const std::vector<Geometry2d::Point>& biasWaypoints = {});

/**
 * Generate a smooth path from start to goal disregarding obstacles.
 */
Trajectory simple(
    const LinearMotionInstant& start, const LinearMotionInstant& goal,
    const MotionConstraints& motionConstraints, RJ::Time startTime,
    const std::vector<Geometry2d::Point>& intermediatePoints = {});

}  // namespace Planning::CreatePath