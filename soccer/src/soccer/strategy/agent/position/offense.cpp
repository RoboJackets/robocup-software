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

void Offense::receive_communication_response(rj_msgs::msg::AgentToPosCommResponse response) {
    // TODO: Do something
}

rj_msgs::msg::PosToAgentCommResponse Offense::receive_communication_request(rj_msgs::msg::AgentToPosCommRequest request) {
    // TODO: Check that request is correct type
    SPDLOG_INFO("\033[92mOFFENSE RECEIVING COMMUNICATION\033[0m");
    // float distance_to_goal = this->world_state()->get_robot(true, robot_id_).pose.position().y();
    float distance_to_goal = 2.25f;
    rj_msgs::msg::GoalLineDistResponse distance_response{};
    distance_response.dist_from_goal_line = distance_to_goal;
    rj_msgs::msg::PosToAgentCommResponse comm_response{};
    comm_response.response.goal_line_dist_response = {distance_response};
    SPDLOG_INFO("\033[92mOFFENSE SENDING COMMUNICATION\033[0m");
    return comm_response;
}

}  // namespace strategy
