#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position{r_id, "Runner"} {}

Runner::Runner(Position&& other) : Position{std::move(other)} { position_name_ = "Runner"; }

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    current_state_ = next_state();
    return state_to_task(std::move(intent));
}

std::string Runner::get_current_state() {
    return std::string{"Runner"} + std::to_string(static_cast<int>(current_state_));
}

Runner::State Runner::next_state() {
    const auto pos = last_world_state_->get_robot(true, robot_id_).pose.position();
    const bool arrived = pos.dist_to(vertices_.at(static_cast<int>(current_state_))) < kArriveDist;

    if (!arrived) {
        return current_state_;
    }

    switch (current_state_) {
        case SIDE_1:
            return SIDE_2;
        case SIDE_2:
            return SIDE_3;
        case SIDE_3:
            return SIDE_4;
        case SIDE_4:
            return SIDE_1;
    }
    return SIDE_1;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    const rj_geometry::Point& target = vertices_.at(static_cast<int>(current_state_));

    intent.motion_command = planning::MotionCommand{
        "path_target", planning::LinearMotionInstant{target}, planning::FaceBall{}};

    return intent;
}

}  // namespace strategy