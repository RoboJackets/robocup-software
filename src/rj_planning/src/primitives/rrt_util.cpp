#include "rj_planning/primitives/rrt_util.hpp"

namespace planning {

using std::vector;
using namespace rj_geometry;

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
                             const vector<Point>& waypoints, bool straight_line,
                             const PlanningConfig& config) {
    auto state_space =
        std::make_shared<RoboCupStateSpace>(FieldDimensions::current_dimensions, obstacles);
    RRT::BiRRT<Point> bi_rrt(state_space, Point::hash, 2);
    bi_rrt.setStartState(start);
    bi_rrt.setGoalState(goal);

    bi_rrt.setStepSize(config.rrt.step_size);
    bi_rrt.setMinIterations(config.rrt.min_iterations);
    bi_rrt.setMaxIterations(config.rrt.max_iterations);
    bi_rrt.setGoalBias(config.rrt.goal_bias);

    if (!waypoints.empty()) {
        bi_rrt.setWaypoints(waypoints);
        bi_rrt.setWaypointBias(config.rrt.waypoint_bias);
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
                           const vector<Point>& waypoints, const PlanningConfig& config) {
    return run_rrt_helper(start, goal, obstacles, waypoints, false, config);
}

}  // namespace planning
