#include "planning/planner/goalie_idle_planner.hpp"

namespace planning {

Trajectory GoalieIdlePlanner::plan(const PlanRequest& plan_request) {
    // lots of this is duplicated from PathTargetPlanner, because there's not
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
    auto command = PathTargetMotionCommand{};
    auto idle_pt = get_idle_pt(plan_request.world_state);
    command.goal.position = idle_pt;

    // Make robot face ball
    auto angle_function = AngleFns::face_point(plan_request.world_state->ball.position);

    // call Replanner to generate a Trajectory
    Trajectory trajectory = Replanner::create_plan(
        Replanner::PlanParams{plan_request.start, command.goal, static_obstacles, dynamic_obstacles,
                              plan_request.constraints, angle_function, RJ::Seconds(3.0)},
        std::move(previous_));

    // Cache current Trajectory, return
    previous_ = trajectory;
    return trajectory;
}

rj_geometry::Point GoalieIdlePlanner::get_idle_pt(const WorldState* world_state) {
    rj_geometry::Point ball_pos = world_state->ball.position;
    // TODO(Kevin): make this depend on team +/-x
    rj_geometry::Point goal_pt{0.0, 0.0};

    double goalie_dist = 0.5;
    rj_geometry::Point idle_pt = (ball_pos - goal_pt).norm();
    idle_pt *= goalie_dist;
    // TODO(Kevin): clamp y to 0 so goalie doesn't go backwards

    return idle_pt;
}

void GoalieIdlePlanner::reset() {}

bool GoalieIdlePlanner::is_done() const { return false; }

}  // namespace planning
