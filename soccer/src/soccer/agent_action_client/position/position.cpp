#include "position.hpp"

namespace strategy {

Position::Position() {}

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
    // mutex lock here as the world state could be accessed while callback from
    // AC is updating it = undefined behavior = crashes
    auto lock = std::lock_guard(world_state_mutex_);
    last_world_state_ = std::move(world_state);
}

[[nodiscard]] WorldState* Position::world_state() {
    // thread-safe getter for world_state (see update_world_state())
    auto lock = std::lock_guard(world_state_mutex_);
    return &last_world_state_;
}

}  // namespace strategy
