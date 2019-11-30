#include "RRTUtil.hpp"
#include <array>
#include <rrt/planning/Path.hpp>
#include "DebugDrawer.hpp"

using namespace Geometry2d;

namespace Planning {

REGISTER_CONFIGURABLE(RRTConfig)

ConfigBool* RRTConfig::EnableRRTDebugDrawing;
ConfigDouble* RRTConfig::StepSize;
ConfigDouble* RRTConfig::GoalBias;
ConfigDouble* RRTConfig::WaypointBias;

ConfigInt* RRTConfig::MinIterations;
ConfigInt* RRTConfig::MaxIterations;

void RRTConfig::createConfiguration(Configuration* cfg) {
    EnableRRTDebugDrawing =
        new ConfigBool(cfg, "PathPlanner/RRT/EnableDebugDrawing", false);
    StepSize = new ConfigDouble(cfg, "PathPlanner/RRT/StepSize", 0.15);
    GoalBias = new ConfigDouble(
        cfg, "PathPlanner/RRT/GoalBias", 0.3,
        "Value from 0 to 1 that determines what proportion of the time the RRT "
        "will grow towards the goal rather than towards a random point");
    WaypointBias = new ConfigDouble(
        cfg, "PathPlanner/RRT/WayPointBias", 0.5,
        "Value from 0 to 1 that determines the portion of the time that the "
        "RRT will"
        " grow towards given waypoints rather than towards a random point");
    MinIterations = new ConfigInt(
            cfg, "PathPlanner/RRT/MinIterations", 10,
            "The minimum number of iterations for running RRT");
    MaxIterations = new ConfigInt(
            cfg, "PathPlanner/RRT/MaxIterations", 200,
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

    for (auto& node : rrt.allNodes()) {
        if (node.parent()) {
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
using Geometry2d::Point;
constexpr double escapeObstacleStepSize = 0.1;
const double escapeObstacleGoalChangeThreshold = Robot_Radius;
std::optional<Point> findNonBlockedPoint(
        Point startPoint, std::shared_ptr<RoboCupStateSpace> state_space,
        int maxItr = 300) {
    if (!state_space->stateValid(startPoint)) {
        RRT::Tree<Point> rrt(state_space, Point::hash, 2);
        rrt.setStartState(startPoint);
        // note: we don't set goal state because we're not looking for a
        // particular point, just something that isn't blocked
        rrt.setStepSize(escapeObstacleStepSize);

        // The starting point is in an obstacle, extend the tree until we find
        // an unobstructed point
        for (int i = 0; i < maxItr; ++i) {
            // extend towards a random point
            RRT::Node<Point>* newNode = rrt.grow();

            // if the new point is not blocked, we're done
            if (newNode && state_space->stateValid(newNode->state())) {
                return newNode->state();
            }
        }
    }
    return std::nullopt;
}
std::vector<Point> GenerateRRT(
        Point start,
        Point goal,
        std::shared_ptr<RoboCupStateSpace> state_space,
        const std::vector<Point>& waypoints,
        std::optional<Point> prevGoal) {
    if(!state_space->stateValid(goal)) {
        std::optional<Point> nonBlockedGoal = findNonBlockedPoint(goal, state_space);
        if(!nonBlockedGoal) return {};
        if(prevGoal) {
            float oldDist = prevGoal->distTo(goal);
            float newDist = nonBlockedGoal->distTo(goal);
            goal = newDist < oldDist - escapeObstacleGoalChangeThreshold
                    ? *nonBlockedGoal
                    : *prevGoal;
        } else {
            goal = *nonBlockedGoal;
        }
    }
    if(state_space->transitionValid(start, goal)) {
        return std::vector<Point>{start, goal};
    }

    RRT::BiRRT<Point> biRRT(state_space, Point::hash, 2);
    biRRT.setStartState(start);
    biRRT.setGoalState(goal);
    biRRT.setStepSize(*RRTConfig::StepSize);
    biRRT.setMinIterations(*RRTConfig::MinIterations);
    biRRT.setMaxIterations(*RRTConfig::MaxIterations);
    biRRT.setGoalBias(*RRTConfig::GoalBias);

    if (!waypoints.empty()) {
        biRRT.setWaypoints(waypoints);
        biRRT.setWaypointBias(*RRTConfig::WaypointBias);
    } else {
        biRRT.setWaypointBias(0);
    }

    bool success = biRRT.run();
    if (!success) {
        return {};
    }

    std::vector<Point> points = biRRT.getPath();

    return points;
}

}
