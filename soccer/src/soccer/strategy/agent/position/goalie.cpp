#include "goalie.hpp"

namespace strategy {

// TODO: lock Goalie id to id given by the ref
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
    if (planning::InterceptPlanner::shot_on_goal_detected(world_state)) {
        /* if (true) { */
        // create PathTargetMotionCommand, set goal position to block_pt
        /*
        auto ptmc = rj_msgs::msg::PathTargetMotionCommand{};
        ptmc.target.position = rj_convert::convert_to_ros(block_pt);

        // make goalie face the ball
        rj_geometry::Point ball_pos = world_state->ball.position;
        auto face_pt = ball_pos;
        ptmc.override_face_point = {rj_convert::convert_to_ros(face_pt)};

        ptmc.ignore_ball = true;  // allow goalie to intersect ball's path

        // update intent
        intent.motion_command.path_target_command = {ptmc};
        intent.motion_command.name = "path_target";
        return intent;
        */

        auto intercept_mc = rj_msgs::msg::InterceptMotionCommand{};
        intent.motion_command.intercept_command = {intercept_mc};
        intent.motion_command.name = "intercept";
        return intent;
    } else {
        // send idle command a few times in case it doesn't get picked up on init
        /* if (send_idle_ct_ < 3) { */
        auto goalie_idle = rj_msgs::msg::GoalieIdleMotionCommand{};
        intent.motion_command.goalie_idle_command = {goalie_idle};
        intent.motion_command.name = "goalie_idle";

        /* send_idle_ct_++; */
        return intent;
        /* } */
    }

    // TODO: capture ball if vel is slow & ball is inside box
    // (waiting on field pts to be given to world_state)

    return std::nullopt;
}

}  // namespace strategy
