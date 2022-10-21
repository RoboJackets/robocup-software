#include "position.hpp"

namespace strategy {

Position::Position() {
    position_name_ = "Position";
    SPDLOG_ERROR("pos name {}", position_name_);
}

rj_msgs::msg::RobotIntent Position::get_task() {
    rj_msgs::msg::RobotIntent intent;
    intent.robot_id = 2;
    auto ptmc = rj_msgs::msg::PathTargetMotionCommand{};

    auto pt = rj_geometry::Point(2.0, 3.0);
    ptmc.target.position = rj_convert::convert_to_ros(pt);
    auto vel = rj_geometry::Point(0.0, 0.0);
    ptmc.target.velocity = rj_convert::convert_to_ros(vel);
    auto face_pt = rj_geometry::Point(1.0, 1.0);
    ptmc.override_face_point = {rj_convert::convert_to_ros(face_pt)};

    intent.motion_command.path_target_command = {ptmc};
    return intent;
}

void Position::tell_done() { is_done_ = true; }

void Position::tell_time_left(RJ::Seconds time_left) { time_left = time_left; }

void Position::tell_goal_canceled() { goal_canceled_ = true; }

}  // namespace strategy
