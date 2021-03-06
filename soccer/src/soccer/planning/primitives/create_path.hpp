#pragma once

#include "planning/motion_constraints.hpp"
#include "planning/trajectory.hpp"
#include "planning/primitives/path_smoothing.hpp"

namespace planning::CreatePath {

/**
 * Generate a smooth path from start to goal avoiding obstacles.
 */
Trajectory rrt(const LinearMotionInstant& start,
               const LinearMotionInstant& goal,
               const MotionConstraints& motion_constraints, RJ::Time start_time,
               const rj_geometry::ShapeSet& static_obstacles,
               const std::vector<DynamicObstacle>& dynamic_obstacles = {},
               const std::vector<rj_geometry::Point>& bias_waypoints = {});

/**
 * Generate a smooth path from start to goal disregarding obstacles.
 */
Trajectory simple(
    const LinearMotionInstant& start, const LinearMotionInstant& goal,
    const MotionConstraints& motion_constraints, RJ::Time start_time,
    const std::vector<rj_geometry::Point>& intermediate_points = {});

}  // namespace planning::CreatePath