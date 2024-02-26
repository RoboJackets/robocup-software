#include "runner.hpp"

#include <spdlog/spdlog.h>

namespace strategy {

Runner::Runner(int r_id) : Position(r_id) {
    position_name_ = "Runner";
    current_state_ = LEFT_SIDE;
}

Runner::State Runner::update_state() {
    State next_state = current_state_;
    rj_geometry::Point robot_position =
        this->last_world_state_->get_robot(true, robot_id_).pose.position();

    if (robot_position.nearly_equals(top_left_target_, 0.1)) {
        next_state = TOP_SIDE;
    } else if (robot_position.nearly_equals(top_right_target_, 0.1)) {
        next_state = RIGHT_SIDE;
    } else if (robot_position.nearly_equals(bottom_right_target_, 0.1)) {
        next_state = BOTTOM_SIDE;
    } else if (robot_position.nearly_equals(bottom_left_target_, 0.1)) {
        next_state = LEFT_SIDE;
    }

    return next_state;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    planning::PathTargetFaceOption face_option = planning::FaceTarget{};
    bool ignore_ball = true;
    rj_geometry::Point target_pt;

    if (current_state_ == LEFT_SIDE) {
        target_pt = top_left_target_;
    } else if (current_state_ == TOP_SIDE) {
        target_pt = top_right_target_;
    } else if (current_state_ == RIGHT_SIDE) {
        target_pt = bottom_right_target_;
    } else if (current_state_ == BOTTOM_SIDE) {
        target_pt = bottom_left_target_;
    } else {
        // should be impossible to reach, but this is equivalent to
        // sending an empty MotionCommand
        return std::nullopt;
    }

    planning::LinearMotionInstant target{target_pt, rj_geometry::Point{0.0, 0.0}};
    intent.motion_command =
        planning::MotionCommand{"path_target", target, face_option, ignore_ball};
    return intent;
}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    current_state_ = update_state();
    return state_to_task(intent);
}

}  // namespace strategy
