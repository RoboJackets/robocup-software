#include "rj_strategy/agent/position/runner.hpp"

#include <utility>

namespace strategy{

Runner::Runner(int r_id) : Position{r_id, "Runner"} {}

Runner::Runner(Position&& other) : Position{std::move(other)} {
    position_name_ = "Runner";
}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    // Get next state, and if different, reset clock
    State new_state = next_state();

    current_state_ = new_state;

    // Calculate task based on state
    return state_to_task(intent);
}

std::string Runner::get_current_state() {
    return std::string{"Runner"} + std::to_string(static_cast<int>(current_state_));
}

Runner::State Runner::next_state() {
    if (!check_is_done()) {
        return current_state_;
    }
    switch(current_state_) {
        case SIDE_ONE: {
            return SIDE_TWO;
        }
        case SIDE_TWO: {
            return SIDE_THREE;
        }
        case SIDE_THREE: {
            return SIDE_FOUR;
        }
        case SIDE_FOUR: {
            return SIDE_ONE;
        }
    }
    return current_state_;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    rj_geometry::Point target;

    switch (current_state_) {
        case SIDE_ONE:
            target = rj_geometry::Point{-2.0, 3.0};
            break;
        case SIDE_TWO:
            target = rj_geometry::Point{2.0, 3.0};
            break;
        case SIDE_THREE:
            target = rj_geometry::Point{2.0, 6.0};
            break;
        case SIDE_FOUR:
            target = rj_geometry::Point{-2.0, 6.0};
            break;
    }

    intent.motion_command = planning::MotionCommand{
        "path_target",
        planning::LinearMotionInstant{target},
        planning::FaceBall{}
    };

    return intent;
}

}