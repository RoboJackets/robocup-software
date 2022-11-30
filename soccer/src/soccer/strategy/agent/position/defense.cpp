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
