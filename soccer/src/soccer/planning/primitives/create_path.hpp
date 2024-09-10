#pragma once

#include <random>

#include "planning/motion_constraints.hpp"
#include "planning/primitives/path_smoothing.hpp"
#include "planning/trajectory.hpp"

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
                        const rj_geometry::ShapeSet& static_obstacles);

std::vector<rj_geometry::Point> get_intermediates(const LinearMotionInstant& start,
                                                  const LinearMotionInstant& goal);

constexpr double kMinScale = 0.5, kMaxScale = 1.5;
constexpr double kMinAngle = 20, kMaxAngle = 140;
constexpr int kNumIntermediates = 10;
constexpr double kStepSize = 0.1;
}  // namespace planning::CreatePath