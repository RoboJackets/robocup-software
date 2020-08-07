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

ConfigBool* RRTConfig::enable_rrt_debug_drawing;
ConfigDouble* RRTConfig::step_size;
ConfigDouble* RRTConfig::goal_bias;
ConfigDouble* RRTConfig::waypoint_bias;

ConfigInt* RRTConfig::min_iterations;
ConfigInt* RRTConfig::max_iterations;

using std::vector;
using namespace Geometry2d;

void RRTConfig::create_configuration(Configuration* cfg) {
    // NOLINTNEXTLINE
    enable_rrt_debug_drawing = new ConfigBool(cfg, "PathPlanner/RRT/EnableDebugDrawing", false);
    // NOLINTNEXTLINE
    step_size = new ConfigDouble(cfg, "PathPlanner/RRT/StepSize", 0.15);
    // NOLINTNEXTLINE
    goal_bias =
        new ConfigDouble(cfg, "PathPlanner/RRT/GoalBias", 0.3,
                         "Value from 0 to 1 that determines what proportion of the time the RRT "
                         "will grow towards the goal rather than towards a random point");
    // NOLINTNEXTLINE
    waypoint_bias =
        new ConfigDouble(cfg, "PathPlanner/RRT/WayPointBias", 0.5,
                         "Value from 0 to 1 that determines the portion of the time that the "
                         "RRT will"
                         " grow towards given waypoints rather than towards a random point");
    // NOLINTNEXTLINE
    min_iterations = new ConfigInt(cfg, "PathPlanner/RRT/MinIterations", 100,
                                   "The minimum number of iterations for running RRT");
    // todo(Ethan) can this be increased? RRT fails sometimes. testing needed
    // //NOLINTNEXTLINE
    max_iterations = new ConfigInt(cfg, "PathPlanner/RRT/MaxIterations", 250,
                                   "The maximum number of iterations for running RRT");
}

ConfigBool enable_expensive_rrt_debug_drawing();

void draw_rrt(const RRT::Tree<Point>& rrt, DebugDrawer* debug_drawer, unsigned shell_id) {
    // Draw each robot's rrts in a different color
    // Note: feel free to change these, they're completely arbitrary
    static const std::array<QColor, 6> colors = {QColor("green"),  QColor("blue"),
                                                 QColor("yellow"), QColor("red"),
                                                 QColor("purple"), QColor("orange")};
    QColor color = colors[shell_id % colors.size()];

    for (const auto& node : rrt.allNodes()) {
        if (node.parent() != nullptr) {
            debug_drawer->draw_line(Segment(node.state(), node.parent()->state()), color,
                                   QString("RobotRRT%1").arg(shell_id));
        }
    }
}

void draw_bi_rrt(const RRT::BiRRT<Point>& bi_rrt, DebugDrawer* debug_drawer, unsigned shell_id) {
    draw_rrt(bi_rrt.startTree(), debug_drawer, shell_id);
    draw_rrt(bi_rrt.goalTree(), debug_drawer, shell_id);
}

vector<Point> run_rrt_helper(Point start, Point goal, const ShapeSet& obstacles,
                             const vector<Point>& waypoints, bool straight_line) {
    auto state_space =
        std::make_shared<RoboCupStateSpace>(Field_Dimensions::current_dimensions, obstacles);
    RRT::BiRRT<Point> bi_rrt(state_space, Point::hash, 2);
    bi_rrt.setStartState(start);
    bi_rrt.setGoalState(goal);

    if (straight_line) {
        // TODO(#1511): Replace this with a check that a straight line doesn't
        // hit any static obstacles. Set the step size to be the distance
        // between the start and goal.
        bi_rrt.setStepSize(state_space->distance(start, goal));
        // Plan straight toward the goal.
        bi_rrt.setGoalBias(1);
        // Try up to five times. If unsuccessful after five tries, there
        // probably doesn't exist
        // a straight path.
        bi_rrt.setMinIterations(0);
        bi_rrt.setMaxIterations(5);
    } else {
        bi_rrt.setStepSize(*RRTConfig::step_size);
        bi_rrt.setMinIterations(*RRTConfig::min_iterations);
        bi_rrt.setMaxIterations(*RRTConfig::max_iterations);
        bi_rrt.setGoalBias(*RRTConfig::goal_bias);

        if (!waypoints.empty()) {
            bi_rrt.setWaypoints(waypoints);
            bi_rrt.setWaypointBias(*RRTConfig::waypoint_bias);
        }
    }

    bool success = bi_rrt.run();
    if (!success) {
        return {};
    }
    vector<Point> points = bi_rrt.getPath();
    RRT::SmoothPath(points, *state_space);
    return std::move(points);
}

vector<Point> generate_rrt(Point start, Point goal, const ShapeSet& obstacles,
                           const vector<Point>& waypoints) {
    // note: we could just use state_space.transition_valid() for the straight
    // line test, but this runs quicker
    vector<Point> straight = run_rrt_helper(start, goal, obstacles, waypoints, true);
    if (!straight.empty()) {
        return std::move(straight);
    }
    return run_rrt_helper(start, goal, obstacles, waypoints, false);
}

}  // namespace Planning
