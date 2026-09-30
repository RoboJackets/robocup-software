#include "rj_planning/planners/escape_obstacles_path_planner.hpp"

using namespace rj_geometry;
namespace planning {

EscapeObstaclesPathPlanner::EscapeObstaclesPathPlanner(rclcpp::Node* node): PathPlanner(node, "halt") {
    this->escape_params_ = build_escape_params();
    this->escape_params_callback_ = node_->add_on_set_parameters_callback(
        [this](const std::vector<rclcpp::Parameter>& params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;
            EscapeParams updated_params = escape_params_;

            for (const auto& param : params) {
                const auto& name = param.get_name();
                if (name != "escape.goal_change_threshold" && name != "escape.step_size") {
                    continue;
                }

                const double value = param.as_double();
                if (!std::isfinite(value) || value < 0.0 ||
                    (name == "escape.step_size" && value == 0.0)) {
                    result.successful = false;
                    result.reason = name == "escape.step_size"
                                        ? "escape.step_size must be finite and positive"
                                        : "escape.goal_change_threshold must be finite and nonnegative";
                    return result;
                }

                if (name == "escape.goal_change_threshold") {
                    updated_params.goal_change_threshold = value;
                } else {
                    updated_params.step_size = value;
                }
            }

            escape_params_ = updated_params;
            return result;
        });
}

EscapeObstaclesPathPlanner::~EscapeObstaclesPathPlanner() {
    if (node_ != nullptr && escape_params_callback_) {
        node_->remove_on_set_parameters_callback(escape_params_callback_.get());
    }
}

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
                                           plan_request.field_dimensions, plan_request.shell_id,
                                         intermediate_params_, rrt_params_);
    plan_angles(&result, start_instant, AngleFns::tangent, plan_request.constraints.rot);
    result.set_debug_text("[ESCAPE " + std::to_string(plan_request.shell_id) + "]");

    previous_target_ = unblocked;

    result.stamp(RJ::now());
    return result;
}

Point EscapeObstaclesPathPlanner::find_non_blocked_goal(Point goal, std::optional<Point> prev_goal,
                                                        const ObstacleSet& obstacles, int max_itr) {
    if (obstacles.obstacle_hit(goal)) {
        auto state_space =
            std::make_shared<RoboCupStateSpace>(FieldDimensions::current_dimensions, obstacles);
        RRT::Tree<Point> rrt(state_space, Point::hash, 2);
        rrt.setStartState(goal);
        // note: we don't set goal state because we're not looking for a
        // particular point, just something that isn't blocked
        rrt.setStepSize(escape_params_.step_size);

        // The starting point is in an obstacle, extend the tree until we find
        // an unobstructed point
        Point new_goal;
        for (int i = 0; i < max_itr; ++i) {
            // extend towards a random point
            RRT::Node<Point>* new_node = rrt.grow();

            // if the new point is not blocked, it becomes the new goal
            if (new_node && !obstacles.obstacle_hit(new_node->state())) {
                new_goal = new_node->state();
                break;
            }
        }

        if (!prev_goal || obstacles.obstacle_hit(*prev_goal)) return new_goal;

        // Only use this newly-found point if it's closer to the desired goal by
        // at least a certain threshold
        float old_dist = (*prev_goal - goal).mag();
        float new_dist = (new_goal - goal).mag();
        if (new_dist + escape_params_.goal_change_threshold < old_dist) {
            return new_goal;
        } else {
            return *prev_goal;
        }
    }
    return goal;
}

bool EscapeObstaclesPathPlanner::is_done() const {
    // Since this is the lowest priority planner, PlannerForRobot automatically
    // switches to a more suitable planner when needed.
    // (see planner_node.cpp)
    return false;
}

EscapeParams EscapeObstaclesPathPlanner::build_escape_params() {
    EscapeParams p;
    p.goal_change_threshold =
        node_->get_parameter_or("escape.goal_change_threshold", 0.9);
    p.step_size =
        node_->get_parameter_or("escape.step_size", 0.1);
    return p;
}

}  // namespace planning
