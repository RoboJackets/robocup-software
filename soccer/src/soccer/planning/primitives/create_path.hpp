#pragma once

#include "planning/motion_constraints.hpp"
#include "planning/primitives/path_smoothing.hpp"
#include "planning/trajectory.hpp"
#include "ros_debug_drawer.hpp"

namespace planning::CreatePath {

/**
 * Generate a smooth path from start to goal avoiding obstacles.
 *
 * Path generated will be one of the following (options listed in preferred order):
 *   1. Straight line to goal.
 *   2. Straight line to a point around the first obstacle, plus a straight
 *   line from that point to the goal. Tightest line that fits this criteria
 *   and doesn't collide with any other obstacles is chosen. (This is
 *   iteratively generated: see the .cpp file for details.)
 *   3. RRT-given path to goal.
 *
 * This is done under the assumption that straight trajectories are more
 * efficient than curves (though technically, all our paths are smoothed to Bezier
 * curves, it seems to be better for motion control).
 *
 * RRT is used as a last resort because it guarantees a path will be found, if
 * one exists. However, the paths it generates are often wider than necessary.
 */
Trajectory rrt(const LinearMotionInstant& start, const LinearMotionInstant& goal,
               const MotionConstraints& motion_constraints, RJ::Time start_time,
               const rj_geometry::ShapeSet& static_obstacles,
               const std::vector<DynamicObstacle>& dynamic_obstacles = {},
               const std::vector<rj_geometry::Point>& bias_waypoints = {});

/**
 * Generate a smooth path from start to goal disregarding obstacles.
 */
Trajectory simple(const LinearMotionInstant& start, const LinearMotionInstant& goal,
                  const MotionConstraints& motion_constraints, RJ::Time start_time,
                  const std::vector<rj_geometry::Point>& intermediate_points = {});

}  // namespace planning::CreatePath
