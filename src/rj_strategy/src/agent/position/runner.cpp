#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {
    Runner::Runner(int r_id) : Position{r_id, "Runner"} {}
    Runner::Runner(const Position& other) : Position{other} {}

    std::string Runner::get_current_state() { 
        return "Runner"; 
    }

    Runner::State Runner::next_state() {
        if (!check_is_done()) {
            return current_state_;
        }

        switch(current_state_) {
            case TOP_LEFT:
                return TOP_RIGHT;
            case TOP_RIGHT:
                return BOTTOM_RIGHT;
            case BOTTOM_RIGHT:
                return BOTTOM_LEFT;
            case BOTTOM_LEFT:
                return TOP_LEFT;
            default:
                return current_state_;
        }
    }

    std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
        current_state_ = next_state();
        return state_to_task(intent);
    }

    std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
        planning::LinearMotionInstant target;
        switch (current_state_) {
            case TOP_LEFT:
                target = planning::LinearMotionInstant{rj_geometry::Point{2.0, 2.5}};
                break;
            case TOP_RIGHT:
                target = planning::LinearMotionInstant{rj_geometry::Point{-2.0, 2.5}};
                break;
            case BOTTOM_RIGHT:
                target = planning::LinearMotionInstant{rj_geometry::Point{-2.0, 6.5}};
                break;
            case BOTTOM_LEFT:
                target = planning::LinearMotionInstant{rj_geometry::Point{2.0, 6.5}};
                break;
        }

        planning::MotionCommand prep_command{"path_target", target, planning::FaceTarget{}};
        intent.motion_command = prep_command;
        return intent;
    }

}
