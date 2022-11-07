#include "offense.hpp"

namespace strategy {

Offense::Offense(int r_id) : Position(r_id) { position_name_ = "Offense"; }

rj_msgs::msg::RobotIntent Offense::get_task() {
    // init an intent with our robot id
    rj_msgs::msg::RobotIntent intent;
    intent.robot_id = robot_id_;

    // if world_state invalid, return empty_intent
    if (!assert_world_state_valid()) {
        return get_empty_intent();
    }

    // FSM: kick -> move away -> repeat
    if (check_is_done()) {
        // switch from kicking -> not kicking or vice versa
        kicking_ = !kicking_;
    }

    // get world_state
    WorldState* world_state = this->world_state();  // thread-safe getter

    // when kicking, send kick
    // otherwise, send move command
    if (kicking_) {
        // TODO(Kevin): line kick is broken, fix it
        /* auto lkmc = rj_msgs::msg::LineKickMotionCommand{}; */
        /* rj_geometry::Point ball_pos = world_state->ball.position; */
        /* lkmc.target = rj_convert::convert_to_ros(ball_pos); */
        /* intent.motion_command.line_kick_command = {lkmc}; */

        auto ptmc = rj_msgs::msg::PathTargetMotionCommand{};
        rj_geometry::Point back_pt{(8.0 * (robot_id_ * 0.1) - 4.0), 6.0};
        ptmc.target.position = rj_convert::convert_to_ros(back_pt);

        // TODO(Kevin): this is still tick based, not event-driven, fix? or clarify somewhere
        // event driven would only get this ball pos once, but this is tick that we pretend is
        // event driven there should be a task for facing the ball always
        rj_geometry::Point ball_pos = world_state->ball.position;
        auto face_pt = ball_pos;
        ptmc.override_face_point = {rj_convert::convert_to_ros(face_pt)};
        ptmc.ignore_ball = true;  // don't try to avoid ball
        intent.motion_command.path_target_command = {ptmc};
    } else {
        auto ptmc = rj_msgs::msg::PathTargetMotionCommand{};
        rj_geometry::Point back_pt{(8.0 * (robot_id_ * 0.1) - 4.0), 2.0};
        ptmc.target.position = rj_convert::convert_to_ros(back_pt);

        rj_geometry::Point ball_pos = world_state->ball.position;
        auto face_pt = ball_pos;
        ptmc.override_face_point = {rj_convert::convert_to_ros(face_pt)};
        ptmc.ignore_ball = false;  // try to avoid ball
        intent.motion_command.path_target_command = {ptmc};
    }

    return intent;
}

}  // namespace strategy
