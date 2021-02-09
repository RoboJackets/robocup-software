#pragma once
#include <debug_drawer.hpp>
#include <rj_geometry/point.hpp>
#include <rj_common/field_dimensions.hpp>
#include <rrt/BiRRT.hpp>

#include "configuration.hpp"
#include "robo_cup_state_space.hpp"
#include "system_state.hpp"
#include "planning/motion_constraints.hpp"
#include "planning/trajectory.hpp"

#include <rj_param_utils/param.hpp>

namespace Planning {
constexpr auto kRRTConfigParamModule = "rrt_config";

DECLARE_NS_BOOL(kRRTConfigParamModule, path_planning::rrt, enable_rrt_debug_drawing)
DECLARE_NS_FLOAT64(kRRTConfigParamModule, path_planning::rrt, step_size)
DECLARE_NS_FLOAT64(kRRTConfigParamModule, path_planning::rrt, goal_bias)
DECLARE_NS_FLOAT64(kRRTConfigParamModule, path_planning::rrt, waypoint_bias)

DECLARE_NS_INT64(kRRTConfigParamModule, path_planning::rrt, min_iterations)
DECLARE_NS_INT64(kRRTConfigParamModule, path_planning::rrt, max_iterations)

class RRTConfig {
public:
};

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
std::vector<rj_geometry::Point> generate_rrt(
    rj_geometry::Point start, rj_geometry::Point goal,
    const rj_geometry::ShapeSet& obstacles,
    const std::vector<rj_geometry::Point>& waypoints = {});

}  // namespace Planning
