#include "intercept_planner.hpp"

#include <rj_constants/constants.hpp>

#include "planning/instant.hpp"
#include "planning/primitives/angle_planning.hpp"
#include "planning/primitives/create_path.hpp"

namespace planning {

Trajectory InterceptPlanner::plan(const PlanRequest& plan_request) {
    SPDLOG_INFO("got here!!");
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
    /* if (static_obstacles.hit(plan_request.start.position())) { */
    /*     reset(); */
    /*     return Trajectory{}; */
    /* } */

    // Create a new PathTargetCommand to fill in with desired block point (see
    // get_block_pt for more details)
    // TODO: make an easily usable PathTargetPlanner (non-templated)
    auto command = PathTargetCommand{};
    auto optional_block_pt = get_block_pt(plan_request.world_state);
    if (!optional_block_pt.has_value()) {
        SPDLOG_INFO("intercept_planner is done");
        is_done_ = true;
        return Trajectory{};
    }

    command.goal.position = optional_block_pt.value();

    // Make robot face ball
    auto angle_function = AngleFns::face_point(plan_request.world_state->ball.position);

    // call Replanner to generate a Trajectory
    Trajectory trajectory = Replanner::create_plan(
        Replanner::PlanParams{plan_request.start, command.goal, static_obstacles, dynamic_obstacles,
                              plan_request.constraints, angle_function, RJ::Seconds(3.0)},
        std::move(previous_));

    // Debug drawing
    if (plan_request.debug_drawer != nullptr) {
        plan_request.debug_drawer->draw_circle(
            rj_geometry::Circle(command.goal.position, static_cast<float>(draw_radius)),
            draw_color);
    }

    // Cache current Trajectory, return
    previous_ = trajectory;
    /* trajectory.set_debug_text("GivingUp"); */
    return trajectory;
}

bool InterceptPlanner::is_done() const { return is_done_; }

std::optional<rj_geometry::Point> InterceptPlanner::get_block_pt(const WorldState* world_state) {
    if (!shot_on_goal_detected(world_state)) {
        is_done_ = true;
        return std::nullopt;
    }

    rj_geometry::Point ball_pos = world_state->ball.position;
    rj_geometry::Point ball_vel = world_state->ball.velocity;

    // find x-coord that the ball would cross on the goal line
    // (0, 0) is our goal, +y points out of goal
    // assume ball vel will remain constant
    // TODO(Kevin): account for acceleration?
    if (ball_vel.y() == 0) {
        return std::nullopt;
    }
    double time_to_cross =
        std::abs(ball_pos.y() / ball_vel.y());  // impossible for ball_vel.y() to be exactly 0
    double cross_x = ball_pos.x() + ball_vel.x() * time_to_cross;

    // if shot is off target, ignore it
    // TODO(Kevin): add field to world_state to avoid hardcoding
    // TODO(Kevin): add this to shot on goal calculations
    if (std::abs(cross_x) > 0.5) {
        is_done_ = true;
        return std::nullopt;
    }

    // otherwise, return point needed to block shot
    rj_geometry::Point block_pt{cross_x, 0.0};  // x coord was solved for y = 0
    // TODO: move the y coord up if possible?
    return block_pt;
}

bool InterceptPlanner::shot_on_goal_detected(const WorldState* world_state) {
    // TODO: make this also check direction of fast ball
    rj_geometry::Point ball_vel = world_state->ball.velocity;
    return ball_vel.mag() > 0.2;
}

}  // namespace planning
