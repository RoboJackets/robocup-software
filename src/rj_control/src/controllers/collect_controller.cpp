#include "rj_control/controllers/collect_controller.hpp"

namespace control {

ControlCommand CollectController::update(
    const WorldState& world_state,
    [[maybe_unused]] const FieldDimensions& field_dimensions,
    const action::Action& action
) {
    if (action.get_type() != id()) {
        return {};
    }

    RobotState state = world_state.get_robot(true, robot_id_);
    const rj_geometry::Point& ball_position = world_state.ball.position;

    ControlCommand command = go_to_pose_.update(
        robot_id_,
        world_state,
        {ball_position, (ball_position - state.pose.position()).angle()},
        x_controller_,
        y_controller_,
        w_controller_
    );

    // Publish Debug Information
    publish_errors();

    double distance = (ball_position - state.pose.position()).mag();
    if (distance < 0.25) {
        command.set_dribble_speed(1.0);
    }

    return command;
}

bool CollectController::complete(
    const WorldState& world_state,
    [[maybe_unused]] const FieldDimensions& field_dimensions,
    const action::Action& action
) {
    if (action.get_type() != id()) {
        return true;
    }

    const rj_geometry::Pose& target = {
        world_state.ball.position,
        (world_state.ball.position - world_state.get_robot(true, robot_id_).pose.position()).angle()
    };
    // TODO (Nathaniel Wert): Use ball sense or something else checking if collect is complete
    return go_to_pose_.complete(
        robot_id_,
        world_state,
        target,
        std::make_pair(
            0.01,
            M_PI / 8
        )
    );
}

} // namespace control