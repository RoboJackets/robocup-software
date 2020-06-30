#include "RRTUtil.hpp"

#include <array>
#include <rrt/planning/Path.hpp>

#include "DebugDrawer.hpp"
#include "PathSmoothing.hpp"
#include "VelocityProfiling.hpp"
#include "planning/Instant.hpp"
#include "planning/MotionConstraints.hpp"
#include "planning/Trajectory.hpp"
#include "planning/TrajectoryUtils.hpp"

namespace Planning {
REGISTER_CONFIGURABLE(RRTConfig)

ConfigBool* RRTConfig::EnableRRTDebugDrawing;
ConfigDouble* RRTConfig::StepSize;
ConfigDouble* RRTConfig::GoalBias;
ConfigDouble* RRTConfig::WaypointBias;

ConfigInt* RRTConfig::MinIterations;
ConfigInt* RRTConfig::MaxIterations;

using std::vector;
using namespace Geometry2d;

void RRTConfig::createConfiguration(Configuration* cfg) {
    // NOLINTNEXTLINE
    EnableRRTDebugDrawing =
        new ConfigBool(cfg, "PathPlanner/RRT/EnableDebugDrawing", false);
    // NOLINTNEXTLINE
    StepSize = new ConfigDouble(cfg, "PathPlanner/RRT/StepSize", 0.15);
    // NOLINTNEXTLINE
    GoalBias = new ConfigDouble(
        cfg, "PathPlanner/RRT/GoalBias", 0.3,
        "Value from 0 to 1 that determines what proportion of the time the RRT "
        "will grow towards the goal rather than towards a random point");
    // NOLINTNEXTLINE
    WaypointBias = new ConfigDouble(
        cfg, "PathPlanner/RRT/WayPointBias", 0.5,
        "Value from 0 to 1 that determines the portion of the time that the "
        "RRT will"
        " grow towards given waypoints rather than towards a random point");
    // NOLINTNEXTLINE
    MinIterations =
        new ConfigInt(cfg, "PathPlanner/RRT/MinIterations", 100,
                      "The minimum number of iterations for running RRT");
    // todo(Ethan) can this be increased? RRT fails sometimes. testing needed
    // //NOLINTNEXTLINE
    MaxIterations =
        new ConfigInt(cfg, "PathPlanner/RRT/MaxIterations", 250,
                      "The maximum number of iterations for running RRT");
}

ConfigBool EnableExpensiveRRTDebugDrawing();

void DrawRRT(const RRT::Tree<Point>& rrt, DebugDrawer* debug_drawer,
             unsigned shellID) {
    // Draw each robot's rrts in a different color
    // Note: feel free to change these, they're completely arbitrary
    static const std::array<QColor, 6> colors = {
        QColor("green"), QColor("blue"),   QColor("yellow"),
        QColor("red"),   QColor("purple"), QColor("orange")};
    QColor color = colors[shellID % colors.size()];

    for (const auto& node : rrt.allNodes()) {
        if (node.parent() != nullptr) {
            debug_drawer->drawLine(
                Segment(node.state(), node.parent()->state()), color,
                QString("RobotRRT%1").arg(shellID));
        }
    }
}

void DrawBiRRT(const RRT::BiRRT<Point>& biRRT, DebugDrawer* debug_drawer,
               unsigned shellID) {
    DrawRRT(biRRT.startTree(), debug_drawer, shellID);
    DrawRRT(biRRT.goalTree(), debug_drawer, shellID);
}

vector<Point> runRRTHelper(Point start, Point goal, const ShapeSet& obstacles,
                           const vector<Point>& waypoints, bool straightLine) {
    auto state_space = std::make_shared<RoboCupStateSpace>(
        Field_Dimensions::Current_Dimensions, obstacles);
    RRT::BiRRT<Point> biRRT(state_space, Point::hash, 2);
    biRRT.setStartState(start);
    biRRT.setGoalState(goal);

    if (straightLine) {
        // TODO(#1511): Replace this with a check that a straight line doesn't
        // hit any static obstacles. Set the step size to be the distance
        // between the start and goal.
        biRRT.setStepSize(state_space->distance(start, goal));
        // Plan straight toward the goal.
        biRRT.setGoalBias(1);
        // Try up to five times. If unsuccessful after five tries, there
        // probably doesn't exist
        // a straight path.
        biRRT.setMinIterations(0);
        biRRT.setMaxIterations(5);
    } else {
        biRRT.setStepSize(*RRTConfig::StepSize);
        biRRT.setMinIterations(*RRTConfig::MinIterations);
        biRRT.setMaxIterations(*RRTConfig::MaxIterations);
        biRRT.setGoalBias(*RRTConfig::GoalBias);

        if (!waypoints.empty()) {
            biRRT.setWaypoints(waypoints);
            biRRT.setWaypointBias(*RRTConfig::WaypointBias);
        }
    }

    bool success = biRRT.run();
    if (!success) {
        return {};
    }
    vector<Point> points = biRRT.getPath();
    RRT::SmoothPath(points, *state_space);
    return std::move(points);
}

vector<Point> GenerateRRT(Point start, Point goal, const ShapeSet& obstacles,
                          const vector<Point>& waypoints) {
    // note: we could just use state_space.transitionValid() for the straight
    // line test, but this runs quicker
    vector<Point> straight =
        runRRTHelper(start, goal, obstacles, waypoints, true);
    if (!straight.empty()) {
        return std::move(straight);
    }
    return runRRTHelper(start, goal, obstacles, waypoints, false);
}

}  // namespace Planning
