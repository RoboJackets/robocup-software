#include "rj_planning/planners/escape_obstacles_path_planner.hpp"

using namespace rj_geometry;
namespace planning {

Trajectory EscapeObstaclesPathPlanner::plan(const PlanRequest& plan_request) {
    const RobotInstant& start_instant = plan_request.start;
    const auto& motion_constraints = plan_request.constraints.mot;

    ObstacleSet obstacles;
    fill_obstacles(plan_request, obstacles, true);

    if (!obstacles.hit(start_instant.position())) {
        // Keep moving, but slow down the current velocity. This allows us to
        // keep continuity when we have short disruptions in planners (i.e.
        // single frame delay).
        Trajectory result{
            {RobotInstant{start_instant.pose, start_instant.velocity * 0.0, start_instant.stamp}}};
        result.mark_angles_valid();
        result.stamp(RJ::now());
        result.set_debug_text("[SLOW " + std::to_string(plan_request.shell_id) + "]");
        return result;
    }

    Point unblocked =
        find_non_blocked_goal(start_instant.position(), previous_target_, obstacles, 300);

    std::optional<Point> opt_prev_pt;

    LinearMotionInstant goal{unblocked, Point()};

    ObstacleSet path_obstacles;
    auto ball_shape =
        std::make_shared<rj_geometry::Circle>(plan_request.world_state->ball.position, kBallRadius);
    path_obstacles.add(std::make_shared<Obstacle>(ball_shape, ball_shape));

    auto result = CreatePath::intermediate(start_instant.linear_motion(), goal, motion_constraints,
                                           start_instant.stamp, path_obstacles,
                                           plan_request.field_dimensions, plan_request.shell_id);
    plan_angles(&result, start_instant, AngleFns::tangent, plan_request.constraints.rot);
    result.set_debug_text("[ESCAPE " + std::to_string(plan_request.shell_id) + "]");

    previous_target_ = unblocked;

    result.stamp(RJ::now());
    return result;
}

Point EscapeObstaclesPathPlanner::find_non_blocked_goal(Point goal, std::optional<Point> prev_goal,
                                                        const ObstacleSet& obstacles, int max_itr) {
    return goal;
}

bool EscapeObstaclesPathPlanner::is_done() const {
    // Since this is the lowest priority planner, PlannerForRobot automatically
    // switches to a more suitable planner when needed.
    // (see planner_node.cpp)
    return false;
}

}  // namespace planning
