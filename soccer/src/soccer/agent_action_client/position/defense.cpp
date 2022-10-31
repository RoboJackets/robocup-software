#include "defense.hpp"

namespace strategy {

Defense::Defense() { position_name_ = "Defense"; }

rj_msgs::msg::RobotIntent Defense::get_task() {
    rj_msgs::msg::RobotIntent intent;
    intent.robot_id = 2;

    // thread-safe getter
    WorldState* world_state = this->world_state();

    if (world_state == nullptr) {
        auto empty = rj_msgs::msg::EmptyMotionCommand{};
        intent.motion_command.empty_command = {empty};
    } else {
        // move to a point
        auto ptmc = rj_msgs::msg::PathTargetMotionCommand{};
        rj_geometry::Point pt{2.0, 3.0};
        ptmc.target.position = rj_convert::convert_to_ros(pt);

        rj_geometry::Point ball_pos = world_state->ball.position;
        auto face_pt = ball_pos;
        ptmc.override_face_point = {rj_convert::convert_to_ros(face_pt)};
        ptmc.ignore_ball = false;
        intent.motion_command.path_target_command = {ptmc};
    }

    return intent;
}

}  // namespace strategy
