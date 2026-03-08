#include "rj_control/controllers/dribble_controller.hpp"

namespace control {

ControlCommand DribbleController::update(
    const WorldState& world_state,
    [[maybe_unused]] const FieldDimensions& field_dimensions,
    const action::Action& action
) {
    if (action.get_type() != action::ActionType::DRIBBLE) {
        return {{0.0, 0.0, 0.0}};
    }

    ControlCommand command;
    if (current_state_ == State::GET_BEHIND_BALL) {
        // Move behind the ball in line with the target
        const rj_geometry::Point& ball_position = world_state.ball.position;
        const rj_geometry::Pose& end_target = action.dribble()->target();
        
        // Move 10 cm behind the ball
        const rj_geometry::Point move_target = ball_position - (end_target.position() - ball_position).norm() * 0.1;
        command = go_to_pose_.update(
            robot_id_,
            world_state,
            {move_target, (ball_position - world_state.get_robot(true, robot_id_).pose.position()).angle()},
            x_controller_,
            y_controller_,
            w_controller_
        );
    } else {
        // Move with the ball towards the target
        // TODO (Nathaniel Wert): Add state to recollect ball if we lose it while dribbling
        command = go_to_pose_.update(
            robot_id_,
            world_state,
            action.dribble()->target(),
            x_controller_,
            y_controller_,
            w_controller_
        );
        command.set_dribble_speed(1.0);
    }

    return command;
}

bool DribbleController::complete(
    const WorldState& world_state,
    [[maybe_unused]] const FieldDimensions& field_dimensions,
    const action::Action& action
) {
    if (action.get_type() != id()) {
        return true;
    }

    if (current_state_ == State::GET_BEHIND_BALL) {
        const rj_geometry::Point& ball_position = world_state.ball.position;
        const rj_geometry::Pose& end_target = action.dribble()->target();

        const rj_geometry::Point move_target = ball_position - (end_target.position() - ball_position).norm() * 0.1;
        if (go_to_pose_.complete(
            robot_id_,
            world_state,
            {move_target, (ball_position - world_state.get_robot(true, robot_id_).pose.position()).angle()},
            std::make_pair(0.1, M_PI / 4)
        )) {
            current_state_ = State::MOVE_WITH_BALL;
        }
        return false;
    }

    return go_to_pose_.complete(
        robot_id_,
        world_state,
        action.dribble()->target(),
        std::make_pair(0.1, M_PI / 4)
    );
}

bool DribbleController::avoid_ball(
    [[maybe_unused]] const WorldState& world_state,
    [[maybe_unused]] const FieldDimensions& field_dimensions,
    [[maybe_unused]] const action::Action& action
) {
    return current_state_ == State::GET_BEHIND_BALL;
}

} // namespace control