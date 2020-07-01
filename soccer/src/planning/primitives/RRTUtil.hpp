#pragma once
#include <DebugDrawer.hpp>
#include <Geometry2d/Point.hpp>
#include <rrt/BiRRT.hpp>
#include <rj_common/Field_Dimensions.hpp>

#include "Configuration.hpp"
#include "RoboCupStateSpace.hpp"
#include "SystemState.hpp"
#include "planning/MotionConstraints.hpp"
#include "planning/Trajectory.hpp"

namespace Planning {
class RRTConfig {
public:
    static void createConfiguration(Configuration* cfg);

    // if set, enables drawng of rrts to the SystemState so they can be shown in
    // the gui
    static ConfigBool* EnableRRTDebugDrawing;

    static ConfigDouble* StepSize;
    static ConfigDouble* GoalBias;
    static ConfigDouble* WaypointBias;

    static ConfigInt* MinIterations;
    static ConfigInt* MaxIterations;
};

/// Drawing
void DrawRRT(const RRT::Tree<Geometry2d::Point>& rrt, DebugDrawer* debug_drawer,
             unsigned shellID);
void DrawBiRRT(const RRT::BiRRT<Geometry2d::Point>& biRRT,
               DebugDrawer* debug_drawer, unsigned shellID);

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
std::vector<Geometry2d::Point> GenerateRRT(
    Geometry2d::Point start, Geometry2d::Point goal,
    const Geometry2d::ShapeSet& obstacles,
    const std::vector<Geometry2d::Point>& waypoints = {});

}  // namespace Planning
