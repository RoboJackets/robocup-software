#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position(r_id, "Runner") {}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    // Runs in a square

    State new_state = next_state();

    if (new_state != this->state_) {
        this->state_ = new_state;
        SPDLOG_INFO("Robot {}: now {}", robot_id_, get_current_state());
    }

    return state_to_task(intent);
}

std::string Runner::get_current_state() {
    switch (this->state_) {
        case IDLING:
            return "IDLE";
        case BOTTOM_LEFT:
            return "BOTTOM_LEFT";
        case TOP_LEFT:
            return "TOP_LEFT";
        case TOP_RIGHT:
            return "TOP_RIGHT";
        case BOTTOM_RIGHT:
            return "BOTTOM_RIGHT";
    }
    return "UNKNOWN";
}

Runner::State Runner::next_state() {
    rj_geometry::Point last_robot_pos =
        last_world_state_->get_robot(true, robot_id_).pose.position();
    switch (this->state_) {
        case IDLING:
            square_center_pos_ = rj_geometry::Point{0.0, 4.5};  // center of the field
            return BOTTOM_LEFT;
        case BOTTOM_LEFT:
            if (last_robot_pos.nearly_equals(get_target_corner(BOTTOM_LEFT), kTolerance_)) {
                return TOP_LEFT;
            }
            break;
        case TOP_LEFT:
            if (last_robot_pos.nearly_equals(get_target_corner(TOP_LEFT), kTolerance_)) {
                return TOP_RIGHT;
            }
            break;
        case TOP_RIGHT:
            if (last_robot_pos.nearly_equals(get_target_corner(TOP_RIGHT), kTolerance_)) {
                return BOTTOM_RIGHT;
            }
            break;
        case BOTTOM_RIGHT:
            if (last_robot_pos.nearly_equals(get_target_corner(BOTTOM_RIGHT), kTolerance_)) {
                return BOTTOM_LEFT;
            }
            break;
        default:
            return state_;
    }
    return state_;
}

rj_geometry::Point Runner::get_target_corner(Runner::State state) {
    switch (state) {
        case Runner::BOTTOM_LEFT:
            return square_center_pos_ + rj_geometry::Point{kSquareSize_ / 2, kSquareSize_ / 2};
        case Runner::TOP_LEFT:
            return square_center_pos_ + rj_geometry::Point{kSquareSize_ / 2, -kSquareSize_ / 2};
        case Runner::TOP_RIGHT:
            return square_center_pos_ + rj_geometry::Point{-kSquareSize_ / 2, -kSquareSize_ / 2};
        case Runner::BOTTOM_RIGHT:
            return square_center_pos_ + rj_geometry::Point{-kSquareSize_ / 2, kSquareSize_ / 2};
        default:
            return square_center_pos_;
    }
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    rj_geometry::Point target_point = get_target_corner(this->state_);

    planning::LinearMotionInstant target{target_point};
    planning::MotionCommand motion_command{"path_target", target,
                                           planning::FacePoint{square_center_pos_}};
    intent.motion_command = motion_command;

    return intent;
}

void Runner::derived_acknowledge_pass() {}

void Runner::derived_pass_ball() {}

void Runner::derived_acknowledge_ball_in_transit() {}

}  // namespace strategy