#include "runner.hpp"

namespace strategy {
Runner::Runner(int r_id) : Position(r_id) { position_name_ = "Runner"; }

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    current_state_ = update_state();
    return state_to_task(intent);
}

Runner::State Runner::update_state() {
    State next_state = current_state_;

    WorldState* world_state = this->world_state();

    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();

    if (current_state_ == SIDE_LEFT && robot_position.y() >= field_dimensions_.length() - 1.1) {
        next_state = SIDE_TOP;
    } else if (current_state_ == SIDE_TOP &&
               robot_position.x() >= field_dimensions_.field_x_right_coord() - 1.1) {
        next_state = SIDE_RIGHT;
    } else if (current_state_ == SIDE_RIGHT && robot_position.y() <= 1.1) {
        next_state = SIDE_BOTTOM;
    } else if (current_state_ == SIDE_BOTTOM && robot_position.x() <= field_dimensions_.field_x_left_coord() + 1.1) {
        next_state = SIDE_LEFT;
    }

    return next_state;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    planning::LinearMotionInstant motion_instance;
    if (current_state_ == SIDE_LEFT) {
        motion_instance = planning::LinearMotionInstant{
            rj_geometry::Point{field_dimensions_.field_x_left_coord() + 1, field_dimensions_.length() - 1}, {0.0, 0.0}};
    } else if (current_state_ == SIDE_TOP) {
        motion_instance = planning::LinearMotionInstant{
            rj_geometry::Point{field_dimensions_.field_x_right_coord() - 1, field_dimensions_.length() - 1}, {0.0, 0.0}};
    } else if (current_state_ == SIDE_RIGHT) {
        motion_instance = planning::LinearMotionInstant{
            rj_geometry::Point{field_dimensions_.field_x_right_coord() - 1,
                                               1}, {0.0, 0.0}};
    } else if (current_state_ == SIDE_BOTTOM) {
        motion_instance = planning::LinearMotionInstant{
            rj_geometry::Point{field_dimensions_.field_x_left_coord() + 1, 1},
            {0.0, 0.0}};
    }
    auto face_ball_cmd = planning::MotionCommand{"path_target", motion_instance, planning::FaceTarget{}, true};
    intent.motion_command = face_ball_cmd;
    return intent;
}

void Runner::derived_acknowledge_pass() {}
void Runner::derived_pass_ball() {}
void Runner::derived_acknowledge_ball_in_transit() {}

}  // namespace strategy
