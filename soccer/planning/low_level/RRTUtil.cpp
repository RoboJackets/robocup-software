#include "RRTUtil.hpp"

#include <array>
#include <rrt/planning/Path.hpp>

#include "DebugDrawer.hpp"
#include "PathSmoothing.hpp"
#include "VelocityProfiling.hpp"
#include "planning/MotionConstraints.hpp"
#include "planning/Trajectory.hpp"
#include "planning/Instant.hpp"

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
    MinIterations =
        new ConfigInt(cfg, "PathPlanner/RRT/MinIterations", 100,
                      "The minimum number of iterations for running RRT");
    // todo(Ethan) can this be increased? RRT fails sometimes. testing needed
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

vector<Point> runRRTHelper(Point start, Point goal, const ShapeSet& obstacles,
                           const vector<Point>& waypoints, bool straightLine) {
    auto state_space = std::make_shared<RoboCupStateSpace>(
        Field_Dimensions::Current_Dimensions, obstacles);
    RRT::BiRRT<Point> biRRT(state_space, Point::hash, 2);
    biRRT.setStartState(start);
    biRRT.setGoalState(goal);

    if (straightLine) {
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

namespace CreatePath {
Trajectory simple(const LinearMotionInstant& start, const LinearMotionInstant& goal,
                  const MotionConstraints& motionConstraints,
                  RJ::Time startTime,
                  const std::vector<Point>& intermediatePoints) {
    std::vector<Point> points;
    points.push_back(start.position);
    for (const Point& pt : intermediatePoints) points.push_back(pt);
    points.push_back(goal.position);
    BezierPath bezier(points, start.velocity, goal.velocity,
                      motionConstraints);
    Trajectory path = ProfileVelocity(bezier, start.velocity.mag(),
                                      goal.velocity.mag(),
                                      motionConstraints, startTime);
    return std::move(path);
}

Trajectory rrt(const LinearMotionInstant& start, const LinearMotionInstant& goal,
               const MotionConstraints& motionConstraints,
               RJ::Time startTime,
               const ShapeSet& static_obstacles,
               const vector<DynamicObstacle>& dynamic_obstacles,
               const vector<Point>& biasWaypoints) {
    if (start.position.distTo(goal.position) < 1e-6) {
        return Trajectory{{
            RobotInstant{
                Pose(start.position, 0),
                Twist(),
                RJ::now()
            }}};
    }
    // maybe we don't need an RRT
    Trajectory straightTrajectory =
        CreatePath::simple(start, goal, motionConstraints, startTime);
    if (start.position.distTo(goal.position) < Robot_Radius ||
        !straightTrajectory.hit(static_obstacles, 0s) &&
            !straightTrajectory.intersects(dynamic_obstacles, startTime)) {
        return std::move(straightTrajectory);
    }

    ShapeSet obstacles = static_obstacles;
    Trajectory path{{}};
    constexpr int attemptsToAvoidDynamics = 10;
    for (int i = 0; i < attemptsToAvoidDynamics; i++) {
        std::vector<Point> points =
            GenerateRRT(start.position, goal.position, obstacles,
                        biasWaypoints);

        BezierPath postBezier(points, start.velocity,
                              goal.velocity, motionConstraints);

        path = ProfileVelocity(postBezier, start.velocity.mag(),
                               goal.velocity.mag(), motionConstraints,
                               startTime);

        Point hitPoint;
        if (path.intersects(dynamic_obstacles, path.begin_time(), &hitPoint)) {
            obstacles.add(
                std::make_shared<Circle>(hitPoint, Robot_Radius * 1.5));
        } else {
            break;
        }
    }
    return std::move(path);
}

Trajectory complete(const LinearMotionInstant& start, const LinearMotionInstant& goal,
                    const MotionConstraints& motionConstraints,
                    RJ::Time startTime,
                    const ShapeSet& static_obstacles,
                    const vector<DynamicObstacle>& dynamic_obstacles,
                    const vector<Point>& biasWaypoints) {
    Trajectory rrtPath =
        CreatePath::rrt(start, goal, motionConstraints, startTime,
                        static_obstacles, dynamic_obstacles, biasWaypoints);
    if (!rrtPath.empty()) {
        return std::move(rrtPath);
    }
    return CreatePath::simple(start, goal, motionConstraints, startTime);
}
}  // namespace CreatePath
}  // namespace Planning
