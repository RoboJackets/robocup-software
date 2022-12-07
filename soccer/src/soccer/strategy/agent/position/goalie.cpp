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

    WorldState* world_state = this->world_state();  // thread-safe getter
    bool needs_to_block = false;
    auto block_pt = get_block_pt(world_state, needs_to_block);

    if (needs_to_block) {
        // create PathTargetMotionCommand, set goal position to block_pt
        auto ptmc = rj_msgs::msg::PathTargetMotionCommand{};
        ptmc.target.position = rj_convert::convert_to_ros(block_pt);

        // make goalie face the ball
        rj_geometry::Point ball_pos = world_state->ball.position;
        auto face_pt = ball_pos;
        ptmc.override_face_point = {rj_convert::convert_to_ros(face_pt)};

        ptmc.ignore_ball = true;  // allow goalie to intersect ball's path

        // update intent
        intent.motion_command.path_target_command = {ptmc};
    } else {
        // send idle command a few times in case it doesn't get picked up on init
        /* if (send_idle_ct_ < 3) { */
        auto goalie_idle = rj_msgs::msg::GoalieIdleMotionCommand{};
        intent.motion_command.goalie_idle_command = {goalie_idle};

        /* send_idle_ct_++; */
        /* return intent; */
        /* } */
    }

    // TODO: capture ball if vel is slow & ball is inside box
    // (waiting on field pts to be given to world_state)

    // TODO(Kevin): make this method std::optional, make AC send nothing on no intent
    return intent;
}

rj_geometry::Point Goalie::get_block_pt(WorldState* world_state, bool& needs_to_block) const {
    // TODO: make intercept planner do what its header file does, so we don't need this
    // also, fix the intercept planner so we don't have to pass in the ball
    // point every tick
    rj_geometry::Point ball_pos = world_state->ball.position;
    rj_geometry::Point ball_vel = world_state->ball.velocity;

    // if ball is slow, doesn't need to block
    if (ball_vel.mag() < 0.1) {
        needs_to_block = false;
        return rj_geometry::Point{-1, -1};  // intentionally invalid
    }

    // find x-coord that the ball would cross on the goal line
    // (0, 0) is our goal, +y points out of goal
    // assume ball vel will remain constant
    double time_to_cross = std::abs(ball_pos.y() / ball_vel.y());  // again, ball_vel is > 0
    double cross_x = ball_pos.x() + ball_vel.x() * time_to_cross;

    // if shot is going out of goal, ignore it
    if (std::abs(cross_x) > 0.6) {  // TODO: add field to world_state to avoid hardcoding
        needs_to_block = false;
        return rj_geometry::Point{-1, -1};  // intentionally invalid
    }

    // otherwise, return point needed to block shot
    needs_to_block = true;
    rj_geometry::Point block_pt{cross_x, 0.0};
    return block_pt;
}

}  // namespace strategy
