#include "goalie.hpp"

namespace strategy {

Goalie::Goalie() {
    position_name_ = "Goalie";
    SPDLOG_INFO("pos name {}", position_name_);
}

rj_msgs::msg::RobotIntent Goalie::get_task() {
    if (check_is_done()) {
        SPDLOG_INFO("goalie says done!");
        move_ct++;
    }

    rj_msgs::msg::RobotIntent intent;
    intent.robot_id = 2;
    auto ptmc = rj_msgs::msg::PathTargetMotionCommand{};

    double x = 2.0;
    if (move_ct % 2 == 1) {
        x = 4.0;
    }

    auto pt = rj_geometry::Point(x, 3.0);
    ptmc.target.position = rj_convert::convert_to_ros(pt);
    auto vel = rj_geometry::Point(0.0, 0.0);
    ptmc.target.velocity = rj_convert::convert_to_ros(vel);
    auto face_pt = rj_geometry::Point(1.0, 1.0);
    ptmc.override_face_point = {rj_convert::convert_to_ros(face_pt)};

    intent.motion_command.path_target_command = {ptmc};
    return intent;
}

}  // namespace strategy
