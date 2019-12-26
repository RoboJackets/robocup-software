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

std::vector<Geometry2d::Point> GenerateRRT(
        Geometry2d::Point start,
        Geometry2d::Point goal,
        std::shared_ptr<RoboCupStateSpace> state_space,
        const std::vector<Geometry2d::Point>& waypoints,
        std::shared_ptr<RRT::BiRRT<Point>> biRRT) {
    if(state_space->transitionValid(start, goal)) {
        return std::vector<Point>{start, goal};
    }
    if(!biRRT) {
        biRRT = std::make_shared<RRT::BiRRT<Point>>(state_space, Point::hash, 2);
    }
    biRRT->setStartState(start);
    biRRT->setGoalState(goal);
    biRRT->setStepSize(*RRTConfig::StepSize);
    biRRT->setMinIterations(*RRTConfig::MinIterations);
    biRRT->setMaxIterations(*RRTConfig::MaxIterations);
    biRRT->setGoalBias(*RRTConfig::GoalBias);

    if (!waypoints.empty()) {
        biRRT->setWaypoints(waypoints);
        biRRT->setWaypointBias(*RRTConfig::WaypointBias);
    } else {
        biRRT->setWaypointBias(0);
    }

    bool success = biRRT->run();
    if (!success) {
        return {};
    }

    std::vector<Point> points = biRRT->getPath();

    return points;
}

Trajectory RRTTrajectory(const RobotInstant& start, const RobotInstant& goal, const MotionConstraints& motionConstraints, const Geometry2d::ShapeSet& static_obstacles, const std::vector<DynamicObstacle>& dynamic_obstacles, const std::vector<Point>& biasWaypoints) {
    Geometry2d::ShapeSet statics = static_obstacles;
    Trajectory path{{}};
    for(int i = 0; i < 10; i++) {
        auto space = std::make_shared<RoboCupStateSpace>(
                Field_Dimensions::Current_Dimensions, statics);
        RJ::Time t0 = RJ::now();
        std::vector<Geometry2d::Point> points = GenerateRRT(
                start.pose.position(), goal.pose.position(),
                space, biasWaypoints);
        double dt = RJ::Seconds(RJ::now() - t0).count();
        t0 = RJ::now();
        //    assert(dt < .005);
        RRT::SmoothPath(points, *space);
        dt = RJ::Seconds(RJ::now() - t0).count();
        t0 = RJ::now();
        assert(dt < .005);
        BezierPath postBezier(points,
                              start.velocity.linear(),
                              goal.velocity.linear(),
                              motionConstraints);
        dt = RJ::Seconds(RJ::now() - t0).count();
        t0 = RJ::now();
        assert(dt < .005);
        path = ProfileVelocity(postBezier,
                                         start.velocity.linear().mag(),
                                         goal.velocity.linear().mag(),
                                         motionConstraints,
                                         start.stamp);
        dt = RJ::Seconds(RJ::now() - t0).count();
        t0 = RJ::now();
        assert(dt < .005);
        Geometry2d::Point hitPoint;
        if(path.intersects(dynamic_obstacles, path.begin_time(), &hitPoint, nullptr)) {
            statics.add(std::make_shared<Circle>(hitPoint, Robot_Radius * 1.5));
        } else {
            break;
        }
    }
    return std::move(path);
}

}
