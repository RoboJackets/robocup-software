#include "goalie.hpp"

namespace strategy {

// TODO: lock Goalie id to id given by the ref
Goalie::Goalie(int r_id) : Position(r_id) { position_name_ = "Goalie"; }

rj_msgs::msg::RobotIntent Goalie::get_task() {
    // init an intent with our robot id
    rj_msgs::msg::RobotIntent intent;
    intent.robot_id = robot_id_;

    // if world_state invalid, return empty_intent
    if (!assert_world_state_valid()) {
        return get_empty_intent();
    }

    if (check_is_done()) {
        move_ct++;
    }

    // thread-safe getter
    WorldState* world_state = this->world_state();

    if (world_state == nullptr) {
        auto empty = rj_msgs::msg::EmptyMotionCommand{};
        intent.motion_command.empty_command = {empty};
    } else {
        auto ptmc = rj_msgs::msg::PathTargetMotionCommand{};
        auto pt = get_block_pt(world_state);
        ptmc.target.position = rj_convert::convert_to_ros(pt);

        rj_geometry::Point ball_pos = world_state->ball.position;
        auto face_pt = ball_pos;
        ptmc.override_face_point = {rj_convert::convert_to_ros(face_pt)};
        ptmc.ignore_ball = true;
        intent.motion_command.path_target_command = {ptmc};
    }
    // TODO: capture ball if vel is slow & ball is inside box
    // (waiting on field pts to be given to world_state)

    return intent;
}

rj_geometry::Point Goalie::get_block_pt(WorldState* world_state) {
    // TODO: make intercept planner do what its header file does, so we don't need this
    // also, fix the intercept planner so we don't have to pass in the ball
    // point every tick
    rj_geometry::Point ball_pos = world_state->ball.position;
    rj_geometry::Point ball_vel = world_state->ball.velocity;

    // if ball is slow, return the idle pt (no kick coming)
    if (ball_vel.mag() < 0.1) {
        return get_idle_pt(world_state);
    }

    // find x-coord that the ball would cross on the goal line
    // (0, 0) is our goal, +y points out of goal
    // assume ball vel will remain constant
    double time_to_cross = std::abs(ball_pos.y() / ball_vel.y());  // again, ball_vel is > 0
    double cross_x = ball_pos.x() + ball_vel.x() * time_to_cross;

    // if shot is going out of goal, ignore it
    // TODO: add field to world_state
    if (std::abs(cross_x) > 0.6) {
        return get_idle_pt(world_state);
    }

    rj_geometry::Point block_pt{cross_x, 0.0};
    return block_pt;
}

rj_geometry::Point Goalie::get_idle_pt(WorldState* world_state) {
    // TODO: transfer field part of world_state
    // TODO: make this depend on team +/-x
    rj_geometry::Point ball_pos = world_state->ball.position;
    rj_geometry::Point ball_vel = world_state->ball.velocity;
    rj_geometry::Point goal_pt{0.0, 0.0};

    // TODO: move closer/farther from ball as a linear % of distance from ball
    double goalie_dist = 0.5;
    rj_geometry::Point idle_pt = (ball_pos - goal_pt).norm();
    idle_pt *= goalie_dist;
    // TODO: clamp y to 0

    return idle_pt;
}

}  // namespace strategy
