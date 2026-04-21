#include "rj_planning/planners/goalie_idle_path_planner.hpp"

namespace planning {

Trajectory GoalieIdlePathPlanner::plan(const PlanRequest& plan_request) {
    // lots of this is duplicated from PathTargetPathPlanner, because there's not
    // an easy way to convert from one PlanRequest to another

    // Collect obstacles
    ObstacleSet obstacles;
    bool ignore_ball = true;
    fill_obstacles(plan_request, obstacles, ignore_ball);

    // If we start inside of an obstacle, give up and let another planner take
    // care of it.
    if (obstacles.hit(plan_request.start.position())) {
        reset();
        return Trajectory();
    }

    // Create a new PathTargetMotionCommand to fill in with desired idle_pt
    auto idle_pt = get_idle_pt(plan_request.world_state, plan_request.shell_id);
    LinearMotionInstant target{idle_pt};

    // Make robot face ball
    auto angle_function = AngleFns::face_point(plan_request.world_state->ball.position);

    // call Replanner to generate a Trajectory
    Trajectory trajectory = Replanner::create_plan(
        Replanner::PlanParams{plan_request.start, target, obstacles, plan_request.field_dimensions,
                              plan_request.constraints, angle_function, plan_request.shell_id,
                              RJ::Seconds(3.0)},
        std::move(previous_));

    // Debug drawing
    if (plan_request.debug_drawer != nullptr) {
        plan_request.debug_drawer->draw_circle(
            rj_geometry::Circle(target.position, static_cast<float>(draw_radius)), draw_color);
    }

    if (!goalie_positions_initialized) {
        left_goal_post = {plan_request.field_dimensions->our_left_goal_post_coordinate().x(), y_distance_from_goal};
        right_goal_post = {plan_request.field_dimensions->our_right_goal_post_coordinate().x(), y_distance_from_goal};
        goalie_positions_initialized = true;
    }
    
    // Cache current Trajectory, return
    previous_ = trajectory;
    return trajectory;
}

rj_geometry::Point GoalieIdlePathPlanner::get_idle_pt(const WorldState* world_state, int goalie_id) {
    const rj_geometry::Point current_position =
        world_state->get_robot(true, goalie_id).pose.position();
        
    static bool moving_towards_left = true;

    goal_target = moving_towards_left ? left_goal_post : right_goal_post;

    // switch directions when close
    if (current_position.dist_to(goal_target) <= tolerance_for_switching) {
        moving_towards_left = !moving_towards_left;
    }

    return goal_target;
}

void GoalieIdlePathPlanner::reset() {}

bool GoalieIdlePathPlanner::is_done() const { return false; }

}  // namespace planning
