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

void Offense::receive_communication_response(rj_msgs::msg::AgentToPosCommResponse response) {
    if (response.response.response_type == 1) {
        SPDLOG_INFO("\033[91mRobot {} has sent a test response with message: {}\033[0m", response.robot_id, response.response.test_response[0].message);
    } else {
        SPDLOG_INFO("\033[93mRobot {} has acknowledged the message: {}\033[0m", response.robot_id);
    }
}

rj_msgs::msg::PosToAgentCommResponse Offense::receive_communication_request(rj_msgs::msg::AgentToPosCommRequest request) {
    rj_msgs::msg::PosToAgentCommResponse comm_response{};
    if (request.request.request_type == 1) {
        rj_msgs::msg::TestResponse test_response{};
        test_response.message = "I have obtained a test request message";
        comm_response.response.test_response = {test_response};
        comm_response.response.response_type = 1;
    } else if (request.request.request_type == 2) {
        rj_msgs::msg::PositionResponse position_response{};
        position_response.position = 2;
        comm_response.response.position_response = {position_response};
        comm_response.response.response_type = 2;
    } else {
        rj_msgs::msg::Acknowledge acknowledge{};
        acknowledge.acknowledged = true;
        comm_response.response.acknowledge_response = {acknowledge};
        comm_response.response.response_type = 4;
    }
    return comm_response;
}

}  // namespace strategy
