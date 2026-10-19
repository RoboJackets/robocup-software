#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position(r_id, "Runner") {}

Runner::Runner(Position&& other) : Position{std::move(other)} {
    position_name_ = "Runner";
}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    // Get next state
    State new_state = next_state();

    // Optionally do timeout stuff

    current_state_ = new_state;

    // Calculate task based on state
    return state_to_task(intent);
}

std::string Runner::get_current_state() {
    return std::string{"Runner"} + std::to_string(static_cast<int>(current_state_));
}

Runner::State Runner::next_state() {
    // check_is_done() is set by the agent when the planner finishes this path
    if (!check_is_done()) return current_state_;

    // handle transitions between current state
    switch(current_state_) {
        case GO_TO_A:
            return GO_TO_B;
        case GO_TO_B:
            return GO_TO_C;
        case GO_TO_C:
            return GO_TO_D;
        case GO_TO_D:
            return GO_TO_A;
    } 

    return current_state_;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    rj_geometry::Point vertex{-0.5, 1.0};

    switch (current_state_) {
        case GO_TO_A: vertex = {-0.5, 1.0}; break;
        case GO_TO_B: vertex = {0.5, 1.0}; break;
        case GO_TO_C: vertex = {0.5, 2.0}; break;
        case GO_TO_D: vertex = {-0.5, 2.0}; break;
    }

    planning::LinearMotionInstant target{vertex};
    intent.motion_command = planning::MotionCommand{"path_target", target, planning::FaceTarget{}};
    return intent;
}


} // namespace strategy