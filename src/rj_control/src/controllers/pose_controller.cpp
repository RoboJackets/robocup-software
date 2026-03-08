#include "rj_control/controllers/pose_controller.hpp"

namespace control {

ControlCommand PoseController::update(
    const WorldState& world_state,
    [[maybe_unused]] const FieldDimensions& field_dimensions,
    const action::Action& action
) {
    if (action.get_type() != id()) {
        return {};
    }
    const rj_geometry::Pose& pose = action.go_to_pose()->target();
    ControlCommand command = go_to_pose_.update(
        robot_id_,
        world_state,
        pose,
        x_controller_,
        y_controller_,
        w_controller_
    );

    publish_errors();

    return command;
}

bool PoseController::complete(
    const WorldState& world_state,
    [[maybe_unused]] const FieldDimensions& field_dimensions,
    const action::Action& action
) {
    if (action.get_type() != id()) {
        return true;
    }

    const rj_geometry::Pose& target = action.go_to_pose()->target();
    return go_to_pose_.complete(
        robot_id_,
        world_state,
        target,
        std::make_pair(
            action.go_to_pose()->position_tolerance(),
            action.go_to_pose()->angular_tolerance()
        )
    );
}

bool PoseController::avoid_ball(
    [[maybe_unused]] const WorldState& world_state,
    [[maybe_unused]] const FieldDimensions& field_dimensions,
    const action::Action& action
) {
    if (action.get_type() != id()) {
        return false;
    }

    return action.go_to_pose()->avoid_ball();
}

} // namespace control