#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position{r_id, "Runner"} {}

Runner::Runner(const Position& other) : Position{other} { position_name_ = "Runner"; }

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    // Get next state, and if different, reset the clock
    State new_state = next_state();

    if (current_state_ != new_state) {
        SPDLOG_INFO("Robot {}: now on side {}", robot_id_, static_cast<int>(new_state) + 1);
    }

    current_state_ = new_state;

    // Calculate task based on state
    return state_to_task(intent);
}

std::string Runner::get_current_state() {
    return std::string{"Runner"} + std::to_string(static_cast<int>(current_state_));
}

Runner::State Runner::next_state() {
    // Advance only once the current straight-line move has finished.
    // SIDE_4 wraps back to SIDE_1, which makes the robot run laps forever.
    switch (current_state_) {
        case SIDE_1:
            return check_is_done() ? SIDE_2 : SIDE_1;
        case SIDE_2:
            return check_is_done() ? SIDE_3 : SIDE_2;
        case SIDE_3:
            return check_is_done() ? SIDE_4 : SIDE_3;
        case SIDE_4:
            return check_is_done() ? SIDE_1 : SIDE_4;
    }
    return SIDE_1;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    // The state's index is the index of the corner we're driving to.
    int i = static_cast<int>(current_state_);

    rj_geometry::Point corner{kCorners[i][0], kCorners[i][1]};
    planning::LinearMotionInstant target{corner};

    intent.motion_command = planning::MotionCommand{"path_target", target, planning::FaceTarget{}};
    return intent;
}

// Runner doesn't take part in passing or communication, so these are no-ops.
void Runner::receive_communication_response(communication::AgentPosResponseWrapper) {}
void Runner::derived_acknowledge_pass() {}
void Runner::derived_pass_ball() {}
void Runner::derived_acknowledge_ball_in_transit() {}

}  // namespace strategy