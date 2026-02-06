#pragma once

#include <optional>
#include <random>

#include <rj_common/field_dimensions.hpp>
#include <rj_common/planning/motion_constraints.hpp>
#include <rj_common/planning/trajectory.hpp>
#include <rj_constants/constants.hpp>

#include "rj_planning/primitives/path_smoothing.hpp"
#include "rj_planning/primitives/rrt_util.hpp"
#include "rj_planning/primitives/velocity_profiling.hpp"
#include "rj_planning/trajectory_utils.hpp"

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

Trajectory intermediate(const LinearMotionInstant& start, const LinearMotionInstant& goal,
                        const MotionConstraints& motion_constraints, RJ::Time start_time,
                        const rj_geometry::ShapeSet& static_obstacles,
                        const std::vector<DynamicObstacle>& dynamic_obstacles,
                        const FieldDimensions* field_dimensions, unsigned int robot_id);

/**
 * Grid-based A* path planner using 4-directional movement (up/down/left/right).
 * Uses Euclidean distance as the heuristic for a generous estimate.
 * Designed to be fast; returns an empty trajectory if no path is found within
 * a limited number of iterations, allowing the caller to fall back to RRT.
 */
Trajectory astar(const LinearMotionInstant& start, const LinearMotionInstant& goal,
                 const MotionConstraints& motion_constraints, RJ::Time start_time,
                 const rj_geometry::ShapeSet& static_obstacles,
                 const FieldDimensions* field_dimensions);

std::vector<rj_geometry::Point> get_intermediates(const LinearMotionInstant& start,
                                                  const LinearMotionInstant& goal,
                                                  unsigned int robot_id);
}  // namespace planning::CreatePath