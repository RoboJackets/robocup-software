#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position{r_id, "Runner"} {}

Runner::Runner(Position&& other) : Position{std::move(other)} {
    position_name_ = "Runner";
}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    current_state_ = next_state();

    // Calculate task based on state
    return state_to_task(intent);
}

std::string Runner::get_current_state() {
    return std::string{"Runner"} + std::to_string(static_cast<int>(current_state_));
}

Runner::State Runner::next_state() {
    // handle transitions between current state
    switch (current_state_) {
        case RUNNING_SIDE1: {
            if (check_is_done()) {
                return RUNNING_SIDE2;
            }
            break;
        }

        case RUNNING_SIDE2: {
            if (check_is_done()) {
                return RUNNING_SIDE3;
            }
            break;
        }

        case RUNNING_SIDE3: {
            if (check_is_done()) {
                return RUNNING_SIDE4;
            }
            break;
        }

        case RUNNING_SIDE4: {
            if (check_is_done()) {
                return RUNNING_SIDE1;
            }
            break;
        }
    }

    return current_state_;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    rj_geometry::Point target;

    switch (current_state_) {
        case RUNNING_SIDE1:
            target = rj_geometry::Point{2.0, 5.0};
            break;

        case RUNNING_SIDE2:
            target = rj_geometry::Point{-2.0, 5.0};
            break;

        case RUNNING_SIDE3:
            target = rj_geometry::Point{-2.0, 3};
            break;

        case RUNNING_SIDE4:
            target = rj_geometry::Point{2, 3};
            break;
    }

    auto motion_command = planning::MotionCommand{
        "path_target", 
        planning::LinearMotionInstant{target, rj_geometry::Point{0.0, 0.0}}, 
        planning::FaceAngle{0},
        true
    };

    intent.motion_command = motion_command;
    return intent;
}

}
