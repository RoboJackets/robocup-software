#include "rj_planning/planners/goalie_idle_path_planner.hpp"

namespace planning {

Trajectory GoalieIdlePathPlanner::plan(const PlanRequest& plan_request) {
    // lots of this is duplicated from PathTargetPathPlanner, because there's not
    // an easy way to convert from one PlanRequest to another

    // Collect obstacles
    rj_geometry::ShapeSet static_obstacles;
    std::vector<DynamicObstacle> dynamic_obstacles;
    Trajectory ball_trajectory;
    bool ignore_ball = true;
    fill_obstacles(plan_request, &static_obstacles, &dynamic_obstacles, ignore_ball,
                   &ball_trajectory);

    // If we start inside of an obstacle, give up and let another planner take
    // care of it.
    if (static_obstacles.hit(plan_request.start.position())) {
        reset();
        return Trajectory();
    }

    // Create a new PathTargetMotionCommand to fill in with desired idle_pt
    auto idle_pt = get_idle_pt(plan_request.world_state, plan_request.field_dimensions);
    LinearMotionInstant target{idle_pt};

    // Make robot face ball
    auto angle_function = AngleFns::face_point(plan_request.world_state->ball.position);

    // call Replanner to generate a Trajectory
    Trajectory trajectory = Replanner::create_plan(
        Replanner::PlanParams{plan_request.start, target, static_obstacles, dynamic_obstacles,
                              plan_request.field_dimensions, plan_request.constraints,
                              angle_function, plan_request.shell_id, RJ::Seconds(3.0)},
        std::move(previous_));

    // Debug drawing
    if (plan_request.debug_drawer != nullptr) {
        plan_request.debug_drawer->draw_circle(
            rj_geometry::Circle(target.position, static_cast<float>(draw_radius)), draw_color);
    }

    // Cache current Trajectory, return
    previous_ = trajectory;
    return trajectory;
}

rj_geometry::Point GoalieIdlePathPlanner::get_idle_pt(const WorldState* world_state, const FieldDimensions* field_dimensions) {
    const rj_geometry::Point current_pos =
        world_state->get_robot(true, 0).pose.position();

    static bool screen_dir = true;
    
    rj_geometry::Point left_pt = {field_dimensions->our_left_goal_post_coordinate().x(), 0.5}; // {1.0, 0.5}; // TODO: make the planner use field dimensions instead of hardcoding
    rj_geometry::Point right_pt = {field_dimensions->our_right_goal_post_coordinate().x(), 0.5}; // {-1.0, 0.5}; // TODO: same as above

    rj_geometry::Point target = screen_dir ? left_pt : right_pt;

    // switch directions when close
    if (current_pos.dist_to(target) <= 0.05) {
        screen_dir = !screen_dir;
    }

    return target;
}

void GoalieIdlePathPlanner::reset() {}

bool GoalieIdlePathPlanner::is_done() const { return false; }

}  // namespace planning
