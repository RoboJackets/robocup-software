#include "defense.hpp"

namespace strategy {

Defense::Defense(int r_id) : Position(r_id) { position_name_ = "Defense"; }

rj_msgs::msg::RobotIntent Defense::get_task() {
    // init an intent with our robot id
    rj_msgs::msg::RobotIntent intent;
    intent.robot_id = robot_id_;

    // if world_state invalid, return empty_intent (filled by assert() call)
    if (!assert_world_state_valid(intent)) {
        return intent;
    }

    bool send_new_intent = false;
    if (check_is_done()) {
        // toggle move pts
        SPDLOG_INFO("robot {} is_done", robot_id_);
        move_ct_++;
        send_new_intent = true;
        SPDLOG_INFO("1 send_new_intent {}", send_new_intent);
    }

    SPDLOG_INFO("2 send_new_intent {}", send_new_intent);
    if (send_new_intent) {
        SPDLOG_INFO("sending new intent");
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
}

}  // namespace strategy
