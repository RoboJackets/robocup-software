#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position{r_id, "Runner"} {}

Runner::Runner(Position&& other) : Position{std::move(other)} { position_name_ = "Runner"; }

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    State new_state = next_state();

    if (new_state != current_state_) {
        command_sent_ = false;
    }
    current_state_ = new_state;

    return state_to_task(std::move(intent));
}

std::string Runner::get_current_state() {
    return std::string{"Runner"} + std::to_string(static_cast<int>(current_state_));
}

Runner::State Runner::next_state() {
    // Only advance once we've sent this side's command AND the planner says it's done.
    const bool finished = command_sent_ && check_is_done();

    switch (current_state_) {
        case SIDE_1:
            return finished ? SIDE_2 : SIDE_1;
        case SIDE_2:
            return finished ? SIDE_3 : SIDE_2;
        case SIDE_3:
            return finished ? SIDE_4 : SIDE_3;
        case SIDE_4:
            return finished ? SIDE_1 : SIDE_4;  // wrap around to repeat the lap
    }
    return SIDE_1;  // unreachable, keeps the compiler happy
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    const rj_geometry::Point& target = vertices_.at(static_cast<int>(current_state_));

    intent.motion_command = planning::MotionCommand{
        "path_target", planning::LinearMotionInstant{target}, planning::FaceBall{}};
    command_sent_ = true;

    return intent;
}

}  // namespace strategy