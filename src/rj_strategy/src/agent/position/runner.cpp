#include "rj_strategy/agent/position/runner.hpp"
#include <utility>

namespace strategy {

Runner::Runner(int robot_id) : Position{robot_id, "Runner"} {}

// Calls the base class move constructor directly, bypassing the protected access issue
Runner::Runner(Position&& other) : Position{std::move(other)} {
    position_name_ = "Runner";
}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    update_state();
    return state_to_task(intent);
}

std::string Runner::get_current_state() {
    return "Runner";
}

void Runner::update_state() {
    if (check_is_done()) {
        switch (current_state_) {
            case State::SIDE_1:
                current_state_ = State::SIDE_2;
                break;
            case State::SIDE_2:
                current_state_ = State::SIDE_3;
                break;
            case State::SIDE_3:
                current_state_ = State::SIDE_4;
                break;
            case State::SIDE_4:
                current_state_ = State::SIDE_1;
                break;
        }
    }
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    rj_geometry::Point target_pt;

    switch (current_state_) {
        case State::SIDE_1:
            target_pt = rj_geometry::Point(1.0, 2.0);
            break;
        case State::SIDE_2:
            target_pt = rj_geometry::Point(1.0, 4.0);
            break;
        case State::SIDE_3:
            target_pt = rj_geometry::Point(-1.0, 4.0);
            break;
        case State::SIDE_4:
            target_pt = rj_geometry::Point(-1.0, 2.0);
            break;
    }

    // Matches the generic MotionCommand string formatting used in offense.cpp
    planning::LinearMotionInstant target{target_pt};
    auto path_target = planning::MotionCommand{"path_target", target, planning::FaceTarget{}};
    intent.motion_command = path_target;
    
    return intent;
}

} // namespace strategy