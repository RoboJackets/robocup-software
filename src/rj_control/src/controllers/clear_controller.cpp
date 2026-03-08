#include "rj_control/controllers/clear_controller.hpp"

namespace control {

void ClearController::start(
    const WorldState& world_state,
    const FieldDimensions& field_dimensions,
    const action::Action& action
) {
    
}

ControlCommand ClearController::update(
    const WorldState& world_state,
    const FieldDimensions& field_dimensions,
    const action::Action& action
) {
    return {};
}

bool ClearController::complete(
    const WorldState& world_state,
    const FieldDimensions& field_dimensions,
    const action::Action& action
) {
    return true;
}

bool ClearController::avoid_ball(
    const WorldState& world_state,
    const FieldDimensions& field_dimensions,
    const action::Action& action
) {
    return false;
}

} // namespace control