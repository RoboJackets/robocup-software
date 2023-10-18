#include "analyzer.hpp"

namespace strategy {

Analyzer::Analyzer(int r_id) : robot_id_(r_id) {}

std::unique_ptr<Position> Analyzer::get_init_position(WorldState& world_state, FieldDimensions& field_dimensions) {
    // Point class variables to parameter references
    field_dimensions_ = field_dimensions;
    last_world_state_ = &world_state;

    // if world_state invalid, return empty MotionCommand (equivalent to HALT)
    if (!assert_world_state_valid()) {
        current_position_ = std::make_unique<Defense>(robot_id_);
        return std::move(current_position_);
    }

    if (robot_id_ == 0) {
        current_position_ = std::make_unique<Goalie>(robot_id_);
    } else if (robot_id_ == 1) {
        current_position_ = std::make_unique<Offense>(robot_id_);
    } else {
        current_position_ = std::make_unique<Defense>(robot_id_);
    }

    return std::move(current_position_);
}

std::optional<RobotIntent> get_task(WorldState& world_state, FieldDimensions& field_dimensions) {

    return current_position_->get_task(world_state, field_dimensions);

}

bool Analyzer::assert_world_state_valid() {
    if (last_world_state_ == nullptr) {
        SPDLOG_WARN("WorldState!");
        return false;
    }
    return true;
}

}  // namespace strategy
