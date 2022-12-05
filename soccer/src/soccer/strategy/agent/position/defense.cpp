#include "defense.hpp"

namespace strategy {

Defense::Defense(int r_id) : Position(r_id) { position_name_ = "Defense"; }

rj_msgs::msg::RobotIntent Defense::get_task() {
    // init an intent with our robot id
    rj_msgs::msg::RobotIntent intent;
    intent.robot_id = robot_id_;

    // if world_state invalid, return empty_intent
    if (!assert_world_state_valid()) {
        return get_empty_intent();
    }

    if (check_is_done()) {
        // toggle move pts
        move_ct_++;
    }

    // thread-safe getter
    WorldState* world_state = this->world_state();

    // oscillate between two points
    auto ptmc = rj_msgs::msg::PathTargetMotionCommand{};
    double x = -3.0;
    if (move_ct_ % 2 == 1) {
        x = 3.0;
    }
    rj_geometry::Point pt{x, 3.0};
    ptmc.target.position = rj_convert::convert_to_ros(pt);

    rj_geometry::Point ball_pos = world_state->ball.position;
    auto face_pt = ball_pos;
    ptmc.override_face_point = {rj_convert::convert_to_ros(face_pt)};
    ptmc.ignore_ball = false;
    intent.motion_command.path_target_command = {ptmc};

    return intent;
}

void Defense::receive_communication_response(rj_msgs::msg::AgentToPosCommResponse response) {
    if (response.response.response_type == 1) {
        SPDLOG_INFO("\033[91mRobot {} has sent the message: {}\033[0m", response.robot_id,
                    response.response.test_response[0].message);
    } else {
        SPDLOG_INFO("\033[93mRobot {} has acknowledged the message: {}\033[0m", response.robot_id);
    }
}

rj_msgs::msg::PosToAgentCommResponse Defense::receive_communication_request(
    rj_msgs::msg::AgentToPosCommRequest request) {
    rj_msgs::msg::PosToAgentCommResponse comm_response{};
    if (request.request.request_type == 1) {
        rj_msgs::msg::TestResponse test_response{};
        test_response.message = "I have obtained you message and I player defense";
        comm_response.response.test_response = {test_response};
        comm_response.response.response_type = 1;
    } else if (request.request.request_type == 2) {
        rj_msgs::msg::PositionResponse position_response{};
        position_response.position = 1;
        comm_response.response.position_response = {position_response};
        comm_response.response.response_type = 2;
    } else {
        rj_msgs::msg::Acknowledge acknowledge{};
        acknowledge.acknowledged = true;
        comm_response.response.acknowledge_response = {acknowledge};
        comm_response.response.response_type = 4;
    }
    // TODO: Remove Below upon approval
    set_test_multicast_request();

    return comm_response;
}

void Defense::set_test_multicast_request() {
    rj_msgs::msg::PosToAgentCommRequest request;
    rj_msgs::msg::TestRequest test_request;
    request.num_targets = 3;
    request.target_agents = {0, 2, 3};
    request.request.test_request = {test_request};
    request.request.request_type = 1;
    communication_request_ = request;
}

}  // namespace strategy
