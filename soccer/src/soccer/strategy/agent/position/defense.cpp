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
    // TODO: do something
}

rj_msgs::msg::PosToAgentCommResponse Defense::receive_communication_request(rj_msgs::msg::AgentToPosCommRequest request) {
    // TODO: Check that request is correct type
    SPDLOG_INFO("\033[92mDEFENSE RECEIVING COMMUNICATION\033[0m");
    float distance_to_goal = this->world_state()->get_robot(true, robot_id_).pose.position().y();
    SPDLOG_INFO("\033[92mGOT DISTANCE TO GOAL\033[0m");
    rj_msgs::msg::GoalLineDistResponse distance_response{};
    distance_response.dist_from_goal_line = distance_to_goal;
    rj_msgs::msg::PosToAgentCommResponse comm_response{};
    comm_response.response.goal_line_dist_response = {distance_response};
    SPDLOG_INFO("\033[92mDEFENSE SENDING COMMUNICATION\033[0m");
    return comm_response;
}

}  // namespace strategy
