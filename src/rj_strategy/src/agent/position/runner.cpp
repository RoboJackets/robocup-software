#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position{r_id, "Runner"} {}

Runner::Runner(Position&& other) : Position{std::move(other)} { position_name_ = "Runner"; }

std::string Runner::get_current_state() {
    return std::string{"Runner: "} + std::string(state_to_name(current_state_));
}

void Runner::next_state() {
    switch (current_state_) {
        case StarState::STAR_1:
            current_state_ = StarState::STAR_2;
            break;
        case StarState::STAR_2:
            current_state_ = StarState::STAR_3;
            break;
        case StarState::STAR_3:
            current_state_ = StarState::STAR_4;
            break;
        case StarState::STAR_4:
            current_state_ = StarState::STAR_5;
            break;
        case StarState::STAR_5:
            current_state_ = StarState::STAR_1;
            break;
    }
}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    // 1. Get current target position based on state
    rj_geometry::Point target_pos;
    switch (current_state_) {
        case StarState::STAR_1:
            target_pos = rj_geometry::Point{-0.95, 1.45};
            break;
        case StarState::STAR_2:
            target_pos = rj_geometry::Point{0.0, 3.5};
            break;
        case StarState::STAR_3:
            target_pos = rj_geometry::Point{0.95, 1.45};
            break;
        case StarState::STAR_4:
            target_pos = rj_geometry::Point{-1.24, 2.65};
            break;
        case StarState::STAR_5:
            target_pos = rj_geometry::Point{1.24, 2.65};
            break;
    }

    // 2. Check distance to target
    auto robot_pos = last_world_state_->get_robot(true, robot_id_).pose.position();
    double dist = (robot_pos - target_pos).mag();

    // 3. Advance to the next point if close enough or check_is_done() flags true
    if (dist < 0.25 || check_is_done()) {
        next_state();
        // Update target_pos immediately to the new state
        switch (current_state_) {
            case StarState::STAR_1:
                target_pos = rj_geometry::Point{-0.95, 1.45};
                break;
            case StarState::STAR_2:
                target_pos = rj_geometry::Point{0.0, 3.5};
                break;
            case StarState::STAR_3:
                target_pos = rj_geometry::Point{0.95, 1.45};
                break;
            case StarState::STAR_4:
                target_pos = rj_geometry::Point{-1.24, 2.65};
                break;
            case StarState::STAR_5:
                target_pos = rj_geometry::Point{1.24, 2.65};
                break;
        }
    }

    // 4. Send motion command
    planning::LinearMotionInstant target{target_pos};
    intent.motion_command = planning::MotionCommand{"path_target", target, planning::FaceTarget{}};

    return intent;
}

}  // namespace strategy