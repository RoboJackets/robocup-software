#include "defense.hpp"

namespace strategy {

Defense::Defense(int r_id) : Position(r_id) { position_name_ = "Defense"; }

std::optional<RobotIntent> Defense::derived_get_task(RobotIntent intent) {
    if (check_is_done()) {
        // toggle move pts
        move_ct_++;
    }

    // thread-safe getter
    WorldState* world_state = this->world_state();

    // oscillate along horizontal line (temp)
    double x = -3.0;
    if (move_ct_ % 2 == 1) {
        x = 3.0;
    }
    rj_geometry::Point target_pt{x, 3.0};

    // stop at end of path
    rj_geometry::Point target_vel{0.0, 0.0};

    // face ball
    planning::PathTargetFaceOption face_option = planning::FaceBall{};

    // avoid ball
    bool ignore_ball = false;

    planning::LinearMotionInstant goal{target_pt, target_vel};
    intent.motion_command = planning::PathTargetMotionCommand{goal, face_option, ignore_ball};
    intent.motion_command_name = "path_target";
    return intent;
}

void Defense::receive_communication_response(communication::AgentPosResponseWrapper response) {
    if (response.response.response_type == communication::CommunicationType::test) {
        SPDLOG_INFO("\033[91mRobot {} has sent the message: {}\033[0m", response.from_robot_id,
                    response.response.test_response[0].message);
    } else {
        SPDLOG_INFO("\033[93mRobot {} has acknowledged the message: {}\033[0m", response.from_robot_id);
    }
}

communication::PosAgentResponseWrapper Defense::receive_communication_request(
    communication::AgentPosRequestWrapper request) {
    communication::PosAgentResponseWrapper comm_response;
    if (request.request.request_type == communication::CommunicationType::test) {
        rj_msgs::msg::TestResponse test_response{};
        test_response.message = "I have obtained you message and I player defense";
        comm_response.response.test_response = {test_response};
        comm_response.response.response_type = communication::CommunicationType::test;
    } else if (request.request.request_type == communication::CommunicationType::position) {
        rj_msgs::msg::PositionResponse position_response{};
        position_response.position = 1;
        comm_response.response.position_response = {position_response};
        comm_response.response.response_type = communication::CommunicationType::position;
    } else {
        rj_msgs::msg::Acknowledge acknowledge{};
        acknowledge.acknowledged = true;
        comm_response.response.acknowledge_response = {acknowledge};
        comm_response.response.response_type = communication::CommunicationType::acknowledge;
    }
    // TODO: Remove Below upon approval
    set_test_multicast_request();

    return comm_response;
}

void Defense::set_test_multicast_request() {
    communication::PosAgentRequestWrapper request;
    rj_msgs::msg::TestRequest test_request;
    request.broadcast = false;
    request.target_agents = {0, 2, 3};
    request.request.test_request = {test_request};
    request.request.request_type = 1;
    communication_request_ = request;
}

}  // namespace strategy
