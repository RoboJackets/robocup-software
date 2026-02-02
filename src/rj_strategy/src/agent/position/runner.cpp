#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position(r_id, "Runner") {}

Runner::Runner(const Position& other) : Position{other} {}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    latest_state_ = update_state();
    return state_to_task(intent);
}

std::string Runner::get_current_state() {
    switch (latest_state_) {
        case MOVE_LEFT:
            return "MOVE LEFT";
        case MOVE_UP_FIELD:
            return "MOVING UP FIELD";
        case MOVE_RIGHT:
            return "MOVING RIGHT";
        case MOVE_DOWN_FIELD:
            return "MOVING DOWN FIELD";
        default:
            return "IDLE";
    }
}

Runner::State Runner::update_state() {
    WorldState* world_state = last_world_state_;

    // get the robots position
    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
    
    rj_geometry::Point current_target = get_corner_point(latest_state_);

    double distance_to_target = robot_position.dist_to(current_target);

    if (distance_to_target == 0 ) {
        switch (latest_state_) {
        case MOVE_LEFT:
            return MOVE_UP_FIELD;
        case MOVE_UP_FIELD:
            return MOVE_RIGHT;
        case MOVE_RIGHT:
            return MOVE_DOWN_FIELD;
        case MOVE_DOWN_FIELD:
            return MOVE_LEFT;
        }
    }

    return latest_state_;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    rj_geometry::Point target_loc = get_corner_point(latest_state_);

    planning::LinearMotionInstant target{target_loc};
    planning::PathTargetFaceOption face_option = planning::FaceTarget{};

    auto path_target_cmd = planning::MotionCommand{"path_target", target, face_option};
    intent.motion_command = path_target_cmd;

    return intent;
}

rj_geometry::Point Runner::get_corner_point(State state) {
    double width = this->field_dimensions_.length() / 3.0;
    double length = this->field_dimensions_.width() / 3.0;

    switch (state) {
        case MOVE_LEFT:
            return rj_geometry::Point{-half_width, half_length};
        case MOVE_UP_FIELD:
            return rj_geometry::Point{half_width, half_length};
        case MOVE_RIGHT:
            return rj_geometry::Point{half_width, -half_length};
        case MOVE_DOWN_FIELD:
            return rj_geometry::Point{-half_width, -half_length};
        default:
            return rj_geometry::Point{0.0, 0.0};
    }
}



}  // namespace strategy
