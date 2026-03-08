#include "rj_control/controllers/pass_controller.hpp"

namespace control {

void PassController::start(
    const WorldState& world_state,
    const FieldDimensions& field_dimensions,
    const action::Action& action
) {

}

ControlCommand PassController::update(
    const WorldState& world_state,
    const FieldDimensions& field_dimensions,
    const action::Action& action
) {
    return {};
}

bool PassController::complete(
    const WorldState& world_state,
    const FieldDimensions& field_dimensions,
    const action::Action& action
) {
    return true;
}

bool PassController::avoid_ball(
    const WorldState& world_state,
    const FieldDimensions& field_dimensions,
    const action::Action& action
) {
    return false;
}

} // namespace control