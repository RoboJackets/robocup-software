#include "rj_control/controllers/position_controller.hpp"

namespace control {

ControlCommand PositionController::update(
    const WorldState& world_state,
    [[maybe_unused]] const FieldDimensions& field_dimensions,
    const action::Action& action
) {
    if (action.get_type() != id()) {
        return {};
    }
    const rj_geometry::Point& position = action.go_to_point()->target();
    ControlCommand command = go_to_position_.update(
        robot_id_,
        world_state,
        position,
        x_controller_,
        y_controller_,
        w_controller_
    );

    publish_errors();

    return command;
}

bool PositionController::complete(
    const WorldState& world_state,
    [[maybe_unused]] const FieldDimensions& field_dimensions,
    const action::Action& action
) {
    if (action.get_type() != id()) {
        return true;
    }
    const rj_geometry::Point& position = action.go_to_point()->target();
    return go_to_position_.complete(
        robot_id_,
        world_state,
        position,
        action.go_to_point()->tolerance()
    );
}

bool PositionController::avoid_ball(
    [[maybe_unused]] const WorldState& world_state,
    [[maybe_unused]] const FieldDimensions& field_dimensions,
    const action::Action& action
) {
    if (action.get_type() != id()) {
        return false;
    }

    return action.go_to_point()->avoid_ball();
}

} // namespace control