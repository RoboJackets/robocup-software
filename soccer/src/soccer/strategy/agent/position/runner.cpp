#include "runner.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position(r_id) { position_name_ = "Runner"; }

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    current_state_ = update_state();
    return state_to_task(intent);
}

Runner::State Runner::update_state() {
    State next_state = current_state_;
    // handle transitions between states
    WorldState* world_state = this->world_state();
    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();

    switch (current_state_) {
        case RIGHT:
            if (robot_position.x() <= field_dimensions_.field_x_left_coord() + 1.1) next_state = UP;
            break;
        case UP:
            if (robot_position.y() <= 1.1) next_state = LEFT;
            break;
        case LEFT:
            if (robot_position.x() >= field_dimensions_.field_x_right_coord() - 1.1)
                next_state = DOWN;
            break;
        case DOWN:
            if (robot_position.y() >= field_dimensions_.length() - 1.1) next_state = RIGHT;
            break;
    }

    return next_state;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    rj_geometry::Point robot_position = world_state()->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point destination;

    switch (current_state_) {
        case RIGHT:
            destination = rj_geometry::Point{field_dimensions_.field_x_left_coord() + 1,
                                             field_dimensions_.length() - 1};
            break;
        case UP:
            destination = rj_geometry::Point{field_dimensions_.field_x_left_coord() + 1, 1};
            break;
        case LEFT:
            destination = rj_geometry::Point{field_dimensions_.field_x_right_coord() - 1, 1};
            break;
        case DOWN:
            destination = rj_geometry::Point{field_dimensions_.field_x_right_coord() - 1,
                                             field_dimensions_.length() - 1};
            break;
    }

    auto goal = planning::LinearMotionInstant{destination, {0.0, 0.0}};
    auto move_direction =
        planning::MotionCommand{"path_target", goal, planning::FaceTarget{}, true};

    intent.motion_command = move_direction;
    return intent;
}

void Runner::derived_acknowledge_pass() {}

void Runner::derived_pass_ball() {}

void Runner::derived_acknowledge_ball_in_transit() {}

}  // namespace strategy
