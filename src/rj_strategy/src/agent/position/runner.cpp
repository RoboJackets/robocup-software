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
    const rj_geometry::Point center = field_dimensions_.center_field_loc();
    switch (s) {
        case SIDE_1:
            return rj_geometry::Point{center.x() - kHalfWidth, center.y() - kHalfHeight};
        case SIDE_2:
            return rj_geometry::Point{center.x() + kHalfWidth, center.y() - kHalfHeight};
        case SIDE_3:
            return rj_geometry::Point{center.x() + kHalfWidth, center.y() + kHalfHeight};
        case SIDE_4:
            return rj_geometry::Point{center.x() - kHalfWidth, center.y() + kHalfHeight};
    }
    return center;
}

Runner::State Runner::next_state() {
    // check_is_done() clears the flag, so call it exactly once per tick
    const bool done = check_is_done();

    const rj_geometry::Point pos = last_world_state_->get_robot(true, robot_id_).pose.position();
    const bool arrived = pos.dist_to(corner(current_state_)) < kArrivedRadius;

    if (done || arrived) {
        // Advance to the next side, wrapping from SIDE_4 back to SIDE_1
        return static_cast<State>((static_cast<int>(current_state_) + 1) % 4);
    }
    return current_state_;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    intent.motion_command =
        planning::MotionCommand{"path_target", planning::LinearMotionInstant{corner(current_state_)},
                                planning::FaceAngle{0}};
    return intent;
}

}  // namespace strategy
