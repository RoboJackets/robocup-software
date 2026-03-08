#include "rj_control/controllers/rotate_controller.hpp"

namespace control {

ControlCommand RotateController::update(
    const WorldState &world_state,
    [[maybe_unused]] const FieldDimensions &field_dimensions,
    const action::Action &action
) {
    if (action.get_type() != id()) {
        return {{0.0, 0.0}};
    }

    return rotate_.update(
        robot_id_,
        world_state,
        action.rotate_to_heading()->heading(),
        x_controller_,
        y_controller_,
        w_controller_
    );
}

bool RotateController::complete(
    const WorldState &world_state,
    [[maybe_unused]] const FieldDimensions &field_dimensions,
    const action::Action &action
) {
    if (action.get_type() != id()) {
        return true;
    }

    return rotate_.complete(
        robot_id_,
        world_state,
        action.rotate_to_heading()->heading(),
        action.rotate_to_heading()->tolerance()
    );
}

} // namespace control