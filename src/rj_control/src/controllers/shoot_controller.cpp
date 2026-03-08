#include "rj_control/controllers/shoot_controller.hpp"

namespace control {

void ShootController::start(
    const WorldState &world_state,
    const FieldDimensions &field_dimensions,
    const action::Action &action
) {

}

ControlCommand ShootController::update(
    const WorldState &world_state,
    const FieldDimensions &field_dimensions,
    const action::Action &action
) {
    return {};
}

bool ShootController::complete(
    const WorldState &world_state,
    const FieldDimensions &field_dimensions,
    const action::Action &action
) {
    return true;
}

bool ShootController::avoid_ball(
    const WorldState &world_state,
    const FieldDimensions &field_dimensions,
    const action::Action &action
) {
    return false;
}

} // namespace control