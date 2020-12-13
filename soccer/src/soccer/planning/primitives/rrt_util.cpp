#include "rrt_util.hpp"

#include <array>

#include <rrt/planning/Path.hpp>

#include "debug_drawer.hpp"
#include "path_smoothing.hpp"
#include "velocity_profiling.hpp"
#include "planning/instant.hpp"
#include "planning/motion_constraints.hpp"
#include "planning/trajectory.hpp"
#include "planning/trajectory_utils.hpp"

namespace Planning {
REGISTER_CONFIGURABLE(RRTConfig)

ConfigBool* RRTConfig::enable_rrt_debug_drawing;
ConfigDouble* RRTConfig::step_size;
ConfigDouble* RRTConfig::goal_bias;
ConfigDouble* RRTConfig::waypoint_bias;

ConfigInt* RRTConfig::min_iterations;
ConfigInt* RRTConfig::max_iterations;

using std::vector;
using namespace rj_geometry;

void RRTConfig::create_configuration(Configuration* cfg) {
    DEFINE_NS_BOOL(kRRTConfigParamModule, rrt::config, enable_rrt_debug_drawing, false, "Whether or not to enable debug drawing");
    // NOLINTNEXTLINE
    DEFINE_NS_FLOAT64(kRRTConfigParamModule, rrt::config, step_size, 0.15, "Step size for the path planner"); //step_size = new ConfigDouble(cfg, "PathPlanner/RRT/StepSize", 0.15);
    // NOLINTNEXTLINE
    DEFINE_NS_FLOAT64(kRRTConfigParamModule, rrt::config, goal_bias, 0.3, "Value from 0 to 1 that determines what proportion of the time the RRT "
                         "will grow towards the goal rather than towards a random point")
    // NOLINTNEXTLINE
    DEFINE_NS_FLOAT64(kRRTConfigParamModule, rrt::config, waypoint_bias, 0.5, "Value from 0 to 1 that determines the portion of the time that the "
                         "RRT will"
                         " grow towards given waypoints rather than towards a random point")
    // NOLINTNEXTLINE
    DEFINE_NS_INT64(kRRTConfigParamModule, rrt::config, min_iterations, 100, "The minimum number of iterations for running RRT");
    // todo(Ethan) can this be increased? RRT fails sometimes. testing needed
    // //NOLINTNEXTLINE
    DEFINE_NS_INT64(kRRTConfigParamModule, rrt::config, max_iterations, 250, "The maximum number of iterations for running RRT");
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
        std::make_shared<RoboCupStateSpace>(FieldDimensions::current_dimensions, obstacles);
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
        bi_rrt.setStepSize(rrt::config::PARAM_step_size);
        bi_rrt.setMinIterations(rrt::config::PARAM_min_iterations);
        bi_rrt.setMaxIterations(rrt::config::PARAM_max_iterations);
        bi_rrt.setGoalBias(rrt::config::PARAM_goal_bias);

        if (!waypoints.empty()) {
            bi_rrt.setWaypoints(waypoints);
            bi_rrt.setWaypointBias(rrt::config::PARAM_waypoint_bias);
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
