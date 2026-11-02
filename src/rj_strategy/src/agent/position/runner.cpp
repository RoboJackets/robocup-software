#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position{r_id, "Runner"} {}

Runner::Runner(Position&& other) : Position{std::move(other)} { position_name_ = "Runner"; }

std::string Runner::get_current_state() {
    return std::string{"Runner"} + std::to_string(static_cast<int>(current_state_));
}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    current_state_ = next_state();
    return state_to_task(std::move(intent));
}

rj_geometry::Point Runner::corner(State s) const {
    const rj_geometry::Point center{kOffsetX, kOffsetY};
    switch (s) {
        case SIDE_1:
            return rj_geometry::Point{-1.5, 1.2};
        case SIDE_2:
            return rj_geometry::Point{1.5, 1.2};
        case SIDE_3:
            return rj_geometry::Point{1.5, 2.8};
        case SIDE_4:
            return rj_geometry::Point{-1.5, 2.8};
    }
    return center;
}

Runner::State Runner::next_state() {
    const bool done = check_is_done();

    const rj_geometry::Point pos = last_world_state_->get_robot(true, robot_id_).pose.position();
    const bool arrived = pos.dist_to(corner(current_state_)) < kArrivedRadius;

    if (done || arrived) {
        return static_cast<State>((static_cast<int>(current_state_) + 1) % 4);
    }
    return current_state_;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    intent.motion_command = planning::MotionCommand{
        "path_target", planning::LinearMotionInstant{corner(current_state_)},
        planning::FaceAngle{0}};
    return intent;
}

}  // namespace strategy
