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
                        const FieldDimensions* field_dimensions, unsigned int robot_id);

std::vector<rj_geometry::Point> get_intermediates(const LinearMotionInstant& start,
                                                  const LinearMotionInstant& goal,
                                                  unsigned int robot_id);
}  // namespace planning::CreatePath