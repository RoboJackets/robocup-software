#pragma once

#include <array>

#include <rj_common/debug_drawer.hpp>
#include <rj_common/field_dimensions.hpp>
#include <rj_common/planning/instant.hpp>
#include <rj_common/planning/motion_constraints.hpp>
#include <rj_common/planning/trajectory.hpp>
#include <rj_geometry/point.hpp>
#include <rj_param_utils/planning/planning_params.hpp>
#include <rj_rrt/BiRRT.hpp>
#include <rj_rrt/planning/Path.hpp>

#include "rj_planning/primitives/path_smoothing.hpp"
#include "rj_planning/primitives/robo_cup_state_space.hpp"
#include "rj_planning/primitives/velocity_profiling.hpp"
#include "rj_planning/trajectory_utils.hpp"

namespace planning {

/// Drawing
void draw_rrt(const RRT::Tree<rj_geometry::Point>& rrt, DebugDrawer* debug_drawer,
             unsigned shell_id);
void draw_bi_rrt(const RRT::BiRRT<rj_geometry::Point>& bi_rrt,
               DebugDrawer* debug_drawer, unsigned shell_id);

/**
 * Generate a path with BiRRT
 *
 * @param start The starting position.
 * @param goal The goal position. (note: goal.stamp is unused)
 * @param obstacles the obstacles to avoid
 * @param waypoints A vector of points from a previous path. The RRT will be
 *      biased towards these points. If empty, they will be unused.
 * @return A vector of points representing some clear path from the start to
 *      the end.
 */
std::vector<rj_geometry::Point> generate_rrt(rj_geometry::Point start, rj_geometry::Point goal,
                                             const ObstacleSet& obstacles,
                                             const std::vector<rj_geometry::Point>& waypoints = {});

}  // namespace planning
