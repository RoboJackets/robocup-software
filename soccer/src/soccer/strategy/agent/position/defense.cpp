#include "defense.hpp"

namespace strategy {

Defense::Defense(int r_id) : Position(r_id) { position_name_ = "Defense"; }

std::optional<rj_msgs::msg::RobotIntent> Defense::get_task() {
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

    // oscillate along horizontal line (temp)
    auto ptmc = rj_msgs::msg::PathTargetMotionCommand{};
    double x = -1.5;
    if (move_ct_ % 2 == 1) {
        x = 1.5;
    }
    rj_geometry::Point pt{x, 3.5};
    ptmc.target.position = rj_convert::convert_to_ros(pt);

    rj_geometry::Point ball_pos = world_state->ball.position;
    auto face_pt = ball_pos;
    ptmc.override_face_point = {rj_convert::convert_to_ros(face_pt)};
    ptmc.ignore_ball = false;
    intent.motion_command.path_target_command = {ptmc};

    return intent;
}

}  // namespace strategy
