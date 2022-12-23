#include "goalie.hpp"

namespace strategy {

// TODO(Kevin): lock Goalie id to id given by the ref
Goalie::Goalie(int r_id) : Position(r_id) { position_name_ = "Goalie"; }

std::optional<rj_msgs::msg::RobotIntent> Goalie::get_task() {
    // init an intent with our robot id
    rj_msgs::msg::RobotIntent intent;
    intent.robot_id = robot_id_;

    // if world_state invalid, return empty_intent
    if (!assert_world_state_valid()) {
        return get_empty_intent();
    }

    WorldState* world_state = this->world_state();  // thread-safe getter
    if (shot_on_goal_detected(world_state)) {
        // TODO(Kevin): fix intercept planner's is_done, then add in logic to
        // clear/pass ball once done intercepting
        auto intercept_mc = rj_msgs::msg::InterceptMotionCommand{};
        intercept_mc.target.x = 0.0;
        intercept_mc.target.y = 0.1;
        intent.motion_command.intercept_command = {intercept_mc};
        intent.motion_command.name = "intercept";
        return intent;
    } else {
        auto goalie_idle = rj_msgs::msg::GoalieIdleMotionCommand{};
        intent.motion_command.goalie_idle_command = {goalie_idle};
        intent.motion_command.name = "goalie_idle";
        return intent;
    }

    return std::nullopt;
}

bool Goalie::shot_on_goal_detected(WorldState* world_state) {
    rj_geometry::Point ball_pos = world_state->ball.position;
    rj_geometry::Point ball_vel = world_state->ball.velocity;

    // find x-coord that the ball would cross on the goal line to figure out if
    // shot is on target ((0, 0) is our goal, +y points out of goal)
    //
    // assumes ball vel will remain constant
    // TODO(Kevin): account for acceleration?
    if (ball_vel.y() == 0) {
        return false;
    }
    double time_to_cross = std::abs(ball_pos.y() / ball_vel.y());
    double cross_x = ball_pos.x() + ball_vel.x() * time_to_cross;

    bool shot_on_target =
        std::abs(cross_x) < 0.5;  // TODO(Kevin): add field to world_state to avoid hardcoding this
    bool ball_is_fast = ball_vel.mag() > 1.0;
    return ball_is_fast && shot_on_target;
}

}  // namespace strategy
