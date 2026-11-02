#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {

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

    switch (current_state_) {
        case LEFT: {
            if (check_is_done()) {
                return UP;
            }
            return LEFT;
        }

        case UP: {
            if (check_is_done()) {
                return RIGHT;
            }
            return UP;
        }

        case RIGHT: {
            if (check_is_done()) {
                return DOWN;
            }
            return RIGHT;
        }

        case DOWN: {
            if (check_is_done()) {
                return LEFT;
            }
            return DOWN;
        }
    }

    return current_state_;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    switch (current_state_) {
        case LEFT: {
            auto motionCmd = planning::MotionCommand{"path_target", planning::LinearMotionInstant{rj_geometry::Point{2.0, 4.0}}, planning::FaceTarget{}};
            intent.motion_command = motionCmd;
            return intent;
        }
        case UP: {
            auto motionCmd = planning::MotionCommand{"path_target", planning::LinearMotionInstant{rj_geometry::Point{-2.0, 4.0}}, planning::FaceTarget{}};
            intent.motion_command = motionCmd;
            return intent;
        }
        case RIGHT: {
            auto motionCmd = planning::MotionCommand{"path_target", planning::LinearMotionInstant{rj_geometry::Point{-2.0, 6.0}}, planning::FaceTarget{}};
            intent.motion_command = motionCmd;
            return intent;
        }
        case DOWN: {
            auto motionCmd = planning::MotionCommand{"path_target", planning::LinearMotionInstant{rj_geometry::Point{2.0, 6.0}}, planning::FaceTarget{}};
            intent.motion_command = motionCmd;
            return intent;
        }
    }

    return intent;
}

}