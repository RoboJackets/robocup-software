#include "runner.hpp"

namespace strategy {

// TOPLEFT: (1.5, 3.0)
// TOPRIGHT: (-1.5, 3.0)
// BOTTOMLEFT: (1.5, 6.0)
// BOTTOMRIGHT: (-1.5, 6.0)

Runner::Runner(int r_id) : Position(r_id) {
    position_name_ = "Runner";
    current_state_ = RUNNING_BOTTOM;
}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    current_state_ = update_state();
    return state_to_task(intent);
}

Runner::State Runner::update_state() {
    State next_state_ = current_state_;
    WorldState* world_state = this->world_state();

    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();

    if (current_state_ == RUNNING_LEFT) {
        double dist = robot_position.dist_to(rj_geometry::Point{1.5, 3.0});
        if (dist < 0.3) {
            next_state_ = RUNNING_TOP;
        }
    } else if (current_state_ == RUNNING_TOP) {
        double dist = robot_position.dist_to(rj_geometry::Point{-1.5, 3.0});
        if (dist < 0.3) {
            next_state_ = RUNNING_RIGHT;
        }
    } else if (current_state_ == RUNNING_RIGHT) {
        double dist = robot_position.dist_to(rj_geometry::Point{-1.5, 6.0});
        if (dist < 0.3) {
            next_state_ = RUNNING_BOTTOM;
        }
    } else if (current_state_ == RUNNING_BOTTOM) {
        double dist = robot_position.dist_to(rj_geometry::Point{1.5, 6.0});
        if (dist < 0.3) {
            next_state_ = RUNNING_LEFT;
        }
    }
    return next_state_;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    WorldState* world_state = this->world_state();
    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
    planning::LinearMotionInstant motion_instance;

    if (current_state_ == RUNNING_LEFT) {
        motion_instance = planning::LinearMotionInstant(rj_geometry::Point{1.5, 3.0});
    } else if (current_state_ == RUNNING_TOP) {
        motion_instance = planning::LinearMotionInstant(rj_geometry::Point{-1.5, 3.0});
    } else if (current_state_ == RUNNING_RIGHT) {
        motion_instance = planning::LinearMotionInstant(rj_geometry::Point{-1.5, 6.0});
    } else if (current_state_ == RUNNING_BOTTOM) {
        motion_instance = planning::LinearMotionInstant(rj_geometry::Point{1.5, 6.0});
    }
    auto motion_cmd =
        planning::MotionCommand{"path_target", motion_instance, planning::FaceTarget{}, true};
    intent.motion_command = motion_cmd;
    return intent;
}

}  // namespace strategy