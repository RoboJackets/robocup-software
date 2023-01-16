#include "offense.hpp"

namespace strategy {

Offense::Offense(int r_id) : Position(r_id) { position_name_ = "Offense"; }

std::optional<rj_msgs::msg::RobotIntent> Offense::get_task() {
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

    // oscillate along vertical line (temp)
    auto ptmc = rj_msgs::msg::PathTargetMotionCommand{};
    double y = 6.0;
    if (!kicking_) {
        y = 1.0;
    }
    rj_geometry::Point back_pt{(6.0 * (robot_id_ * 0.1) - 2.0), y};
    ptmc.target.position = rj_convert::convert_to_ros(back_pt);

    rj_geometry::Point ball_pos = world_state->ball.position;
    auto face_pt = ball_pos;
    ptmc.override_face_point = {rj_convert::convert_to_ros(face_pt)};
    ptmc.ignore_ball = true;  // don't try to avoid ball
    intent.motion_command.path_target_command = {ptmc};

    /*
    // TODO(Kevin): line kick is broken, fix it
    if (kicking_) {
        auto lkmc = rj_msgs::msg::LineKickMotionCommand{};
        rj_geometry::Point ball_pos = world_state->ball.position;
        lkmc.target = rj_convert::convert_to_ros(ball_pos);
        intent.motion_command.line_kick_command = {lkmc};

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
    */

    return intent;
}

void Offense::receive_communication_response(communication::AgentPosResponseWrapper response) {
    if (response.response.response_type == communication::CommunicationType::test) {
        SPDLOG_INFO("\033[91mRobot {} has sent a test response with message: {}\033[0m",
                    response.from_robot_id, response.response.test_response[0].message);
    } else {
        SPDLOG_INFO("\033[93mRobot {} has acknowledged the message: {}\033[0m", response.from_robot_id);
    }
}

communication::PosAgentResponseWrapper Offense::receive_communication_request(
    communication::AgentPosRequestWrapper request) {
    communication::PosAgentResponseWrapper comm_response;
    if (request.request.request_type == communication::CommunicationType::test) {
        rj_msgs::msg::TestResponse test_response{};
        test_response.message = "I have obtained a test request message";
        comm_response.response.test_response = {test_response};
        comm_response.response.response_type = communication::CommunicationType::test;
    } else if (request.request.request_type == communication::CommunicationType::position) {
        rj_msgs::msg::PositionResponse position_response{};
        position_response.position = 2;
        comm_response.response.position_response = {position_response};
        comm_response.response.response_type = communication::CommunicationType::position;
    } else {
        rj_msgs::msg::Acknowledge acknowledge{};
        acknowledge.acknowledged = true;
        comm_response.response.acknowledge_response = {acknowledge};
        comm_response.response.response_type = communication::CommunicationType::acknowledge;
    }
    return comm_response;
}

}  // namespace strategy
