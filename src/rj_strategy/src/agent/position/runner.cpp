#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position(r_id, "Runner") {}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    current_state_ = next_state();
    return state_to_task(intent);
}

int Runner::next_state() {
    return states_[(current_state_ + 1) % 4];
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    rj_geometry::Point target_point = corners_[current_state_];
    rj_geometry::Point target_vel{1.0, 1,0};
    planning::PathTargetFaceOption face_option{planning::FaceBall{}};
    bool ignore_ball{true};
    planning::LinearMotionInstant target{target_point, target_vel};
    intent.motion_command = planning::MotionCommand{"path_target", target, face_option, ignore_ball};
    return intent;
}

std::string Runner::get_current_state() { return "Runner"; }

}  // namespace strategy