#include "analyzer.hpp"

namespace strategy {

Analyzer::Analyzer(int r_id) : robot_id_(r_id) {}

std::optional<RobotIntent> Analyzer::get_behavior(WorldState& world_state, FieldDimensions& field_dimensions) {
    // Point class variables to parameter references
    field_dimensions_ = field_dimensions;
    last_world_state_ = world_state;

    // if world_state invalid, return empty MotionCommand (equivalent to HALT)
    if (!assert_world_state_valid()) {
        best_position_ = std::make_unique<Defense>(robot_id_);
        return best_position;
    }

    if (robot_id_ == 0) {
        best_position_ = std::make_unique<Goalie>(robot_id_);
    } else if (robot_id_ == 1) {
        best_position_ = std::make_unique<Offense>(robot_id_);
    } else {
        best_position_ = std::make_unique<Defense>(robot_id_);
    }

    return best_position_;

}

[[nodiscard]] WorldState* Analyzer::world_state() {
    // thread-safe getter for world_state (see update_world_state())
    auto lock = std::lock_guard(world_state_mutex_);
    return &last_world_state_;
}

bool Analyzer::assert_world_state_valid() {
    WorldState* world_state = this->world_state();  // thread-safe getter
    if (world_state == nullptr) {
        SPDLOG_WARN("WorldState!");
        return false;
    }
    return true;
}

}  // namespace strategy
