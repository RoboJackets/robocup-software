#include "rj_strategy/agent/position.hpp"


namespace strategy {

Runner::Runner(int r_id) : Position(r_id, "Runner") {}

Runner::Runner(Position&& other) : Position{std::move(other)} {
    position_name_ = "Runner";
}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    current_state_ = next_state();

    return state_to_task(intent);
}

std::string Runner::get_current_state() {
    return "Runner";
}

Runner::State Runner::next_state() {
    if (!check_is_done()) {
        return current_state_;
    }

    switch (current_state_) {
        case SIDE1: {
            return SIDE2;
        }

        case SIDE2: {
            return SIDE3;
        }

        case SIDE3: {
            return SIDE4;
        }

        case SIDE4: {
            return SIDE1;
        }

        default: {
            return current_state_;
        }
    }
}


std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    switch (current_state_) {
        case SIDE1: {
            intent.motion_command = planning::MotionCommand{"path_target", planning::LinearMotionInstant(
                rj_geometry::Point{3.0, 0.0}
            )};

            return intent;
        }

        case SIDE2: {
            intent.motion_command = planning::MotionCommand{"path_target", planning::LinearMotionInstant(
                rj_geometry::Point{3.0, 9.0}
            )};

            return intent;
        }

        case SIDE3: {
            intent.motion_command = planning::MotionCommand{"path_target", planning::LinearMotionInstant(
                rj_geometry::Point{-3.0, 9.0}
            )};

            return intent;
        }

        case SIDE4: {
            intent.motion_command = planning::MotionCommand{"path_target", planning::LinearMotionInstant(
                rj_geometry::Point{-3.0, 0.0}
            )};

            return intent;
        }

        default: {
            return intent;
        }
    }
}

}