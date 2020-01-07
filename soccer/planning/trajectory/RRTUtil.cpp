#include "RRTUtil.hpp"
#include <array>
#include <rrt/planning/Path.hpp>
#include "DebugDrawer.hpp"
#include "planning/MotionConstraints.hpp"
#include "planning/trajectory/Trajectory.hpp"
#include "planning/trajectory/PathSmoothing.hpp"
#include "planning/trajectory/VelocityProfiling.hpp"

using namespace Geometry2d;

namespace Planning {
//todo(Ethan) fix this
REGISTER_CONFIGURABLE(RRTConfig)

ConfigBool* RRTConfig::EnableRRTDebugDrawing;
ConfigDouble* RRTConfig::StepSize;
ConfigDouble* RRTConfig::GoalBias;
ConfigDouble* RRTConfig::WaypointBias;

ConfigInt* RRTConfig::MinIterations;
ConfigInt* RRTConfig::MaxIterations;

using std::vector;
using Geometry2d::Point;

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
            cfg, "PathPlanner/RRT/MinIterations", 100,
            "The minimum number of iterations for running RRT");
    MaxIterations = new ConfigInt(
            cfg, "PathPlanner/RRT/MaxIterations", 250,
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

vector<Point> runRRTHelper(
        Point start,
        Point goal,
        const Geometry2d::ShapeSet& obstacles,
        const vector<Point>& waypoints,
        bool straightLine) {
    auto state_space = std::make_shared<RoboCupStateSpace>(Field_Dimensions::Current_Dimensions, obstacles);
    RRT::BiRRT<Point> biRRT(state_space, Point::hash, 2);
    biRRT.setStartState(start);
    biRRT.setGoalState(goal);

    if(straightLine) {
        // Set the step size to be the distance between the start and goal.
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

vector<Point> GenerateRRT(
        Point start,
        Point goal,
        const Geometry2d::ShapeSet& obstacles,
        const vector<Point>& waypoints) {
//    printf("runRRT (%.2f, %.2f) -> (%.2f, %.2f)\n", start.x(), start.y(), goal.x(), goal.y());
    // note: we could just use state_space.transitionValid() for the straight
    // line test, but this runs quicker
    vector<Point> straight = runRRTHelper(start, goal, obstacles, waypoints, true);
    if(!straight.empty()) {
        return std::move(straight);
    }
    return runRRTHelper(start, goal, obstacles, waypoints, false);
}

Trajectory RRTTrajectory(const RobotInstant& start, const RobotInstant& goal, const MotionConstraints& motionConstraints, const Geometry2d::ShapeSet& static_obstacles, const vector<DynamicObstacle>& dynamic_obstacles, const vector<Point>& biasWaypoints) {
    if (start.pose.position().distTo(goal.pose.position()) < 1e-6) {
        return Trajectory{{RobotInstant{start.pose, Twist(), RJ::now()}}};
    }
    Geometry2d::ShapeSet obstacles = static_obstacles;
    Trajectory path{{}};
    constexpr int attemptsToAvoidDynamics = 10;
    for(int i = 0; i < attemptsToAvoidDynamics; i++) {
        std::vector<Geometry2d::Point> points = GenerateRRT(
                start.pose.position(), goal.pose.position(),
                obstacles, biasWaypoints);

        BezierPath postBezier(points,
                              start.velocity.linear(),
                              goal.velocity.linear(),
                              motionConstraints);

        path = ProfileVelocity(postBezier,
                                         start.velocity.linear().mag(),
                                         goal.velocity.linear().mag(),
                                         motionConstraints,
                                         start.stamp);

        Geometry2d::Point hitPoint;
        if(path.intersects(dynamic_obstacles, path.begin_time(), &hitPoint, nullptr)) {
            obstacles.add(std::make_shared<Circle>(hitPoint, Robot_Radius * 1.5));
        } else {
            break;
        }
    }
    return std::move(path);
}

Point projectPointIntoField(Point targetPoint, const Geometry2d::Rect& fieldRect, Point ballPoint){
    auto intersectReturn = fieldRect.intersects(Geometry2d::Segment(ballPoint, targetPoint));

    bool validIntersect = std::get<0>(intersectReturn);
    std::vector<Point> intersectPts = std::get<1>(intersectReturn);

    // If the ball intersects the field at some point
    // Just get the intersect point as the new target
    if (validIntersect) {
        // Sorts based on distance to intercept target
        // The closest one is the intercept point which the ball moves
        // through leaving the field Not the one on the other side of the
        // field
        // Choose a point just inside the field
        targetPoint = *std::min_element(intersectPts.begin(), intersectPts.end(), [&](Point a, Point b) {
            return (a - targetPoint).mag() <
                   (b - targetPoint).mag();
        });

        // Doesn't intersect
        // project the ball into the field
    } else {
        // Simple projection
        targetPoint.x() = std::clamp(targetPoint.x(),
                                     (double)fieldRect.minx(), (double)fieldRect.maxx());
        targetPoint.y() = std::clamp(targetPoint.y(),
                                     (double)fieldRect.miny(), (double)fieldRect.maxy());
    }
    return targetPoint;
}

}
