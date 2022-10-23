#include "position.hpp"

namespace strategy {

Position::Position() {
    position_name_ = "Position";
    SPDLOG_INFO("pos name {}", position_name_);
}

void Position::tell_is_done() { is_done_ = true; }

void Position::tell_time_left(double time_left) { time_left_ = time_left; }

// TODO: use goal_canceled_ info
void Position::tell_goal_canceled() { goal_canceled_ = true; }

bool Position::check_is_done() {
    if (is_done_) {
        is_done_ = false;
        return true;
    }
    return false;
}

bool Position::check_goal_canceled() {
    if (goal_canceled_) {
        goal_canceled_ = false;
        return true;
    }
    return false;
}

void Position::update_world_state(WorldState world_state) {
    latest_world_state_ = world_state;
}

}  // namespace strategy
