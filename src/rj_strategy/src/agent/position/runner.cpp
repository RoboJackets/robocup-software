#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position(r_id, "Runner") {}

Runner::Runner(const Position& other) : Position{other} {}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    latest_state_ = update_state();
    return state_to_task(intent);
}

std::string Runner::get_current_state() { return "Runner"; }

Runner::State Runner::update_state() {
    WorldState* world_state = last_world_state_;

    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();

    // if position within threshold of corner channge state

    if (latest_state_ == WALL1) {
        if (robot_position.dist_to(Runner::Point1) < threshold_) {
            return WALL2;
        }
    } else if (latest_state_ == WALL2) {
        if (robot_position.dist_to(Runner::Point2) < Runner::threshold_) {
            return WALL3;
        }
    } else if (latest_state_ == WALL3) {
        if (robot_position.dist_to(Runner::Point3) < Runner::threshold_) {
            return WALL4;
        }
    } else if (latest_state_ == WALL4) {
        if (robot_position.dist_to(Runner::Point4) < Runner::threshold_) {
            return WALL1;
        }
    }

    return latest_state_;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    planning::PathTargetFaceOption face_option = planning::FacePoint{rj_geometry::Point{0.0, 4.5}};
    if (latest_state_ == WALL1) {
        planning::LinearMotionInstant target{Runner::Point1};
        intent.motion_command = planning::MotionCommand{"path_target", target, face_option, true};
        return intent;
    } else if (latest_state_ == WALL2) {
        planning::LinearMotionInstant target{Runner::Point2};
        intent.motion_command = planning::MotionCommand{"path_target", target, face_option, true};
        return intent;
    } else if (latest_state_ == WALL3) {
        planning::LinearMotionInstant target{Runner::Point3};
        intent.motion_command = planning::MotionCommand{"path_target", target, face_option, true};
        return intent;
    } else if (latest_state_ == WALL4) {
        planning::LinearMotionInstant target{Runner::Point4};
        intent.motion_command = planning::MotionCommand{"path_target", target, face_option, true};
        return intent;
    }

    return intent;
}

}  // namespace strategy
