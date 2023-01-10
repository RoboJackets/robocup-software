#include "offense.hpp"

namespace strategy {

Offense::Offense(int r_id) : Position(r_id) { position_name_ = "Offense"; }

std::optional<RobotIntent> Offense::derived_get_task(RobotIntent intent) {
    // FSM: kick -> move away -> repeat
    if (check_is_done()) {
        // switch from kicking -> not kicking or vice versa
        kicking_ = !kicking_;
    }

    // get world_state
    WorldState* world_state = this->world_state();  // thread-safe getter

    // oscillate along vertical line (temp)
    double y = 6.0;
    if (!kicking_) {
        y = 1.0;
    }
    rj_geometry::Point target_pt{(6.0 * (robot_id_ * 0.1) - 2.0), y};

    // stop at end of path
    rj_geometry::Point target_vel{0.0, 0.0};

    // face ball on way up, face path on way down
    planning::PathTargetFaceOption face_option = planning::FaceBall{};
    if (kicking_) {
        face_option = planning::FaceTarget{};
    }

    // avoid ball
    bool ignore_ball = false;

    planning::LinearMotionInstant goal{target_pt, target_vel};
    intent.motion_command = planning::PathTargetMotionCommand{goal, face_option, ignore_ball};
    intent.motion_command_name = "path_target";
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
