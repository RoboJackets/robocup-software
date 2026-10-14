#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position(r_id, "Runner") {}

Runner::Runner(Position&& other) : Position{std::move(other)} {
    position_name_ = "Runner";
}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    current_state_ = update_state();

    return state_to_task(intent);
}

std::string Runner::get_current_state() {
    return std::string{"Runner"} + std::to_string(static_cast<int>(current_state_));
}

Runner::State Runner::update_state() {
    WorldState* world_state = last_world_state_;

    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();

    // Update state based on coordinator async calls resolving
    State next_state = current_state_;

    switch (current_state_) {
        case TOP_LEFT:
            double distance_to_top_right = robot_position.dist_to(top_right_);
            if (check_is_done() || distance_to_top_right < 0.1) {
                next_state = TOP_RIGHT;
            }
            break;
        case TOP_RIGHT:
            double distance_to_bottom_right = robot_position.dist_to(bottom_right_);
            if (check_is_done() || distance_to_bottom_right < 0.1) {
                next_state = BOTTOM_RIGHT;
            }
            break;
        case BOTTOM_RIGHT:
            double distance_to_bottom_left = robot_position.dist_to(bottom_left_);
            if (check_is_done() || distance_to_bottom_left < 0.1) {
                next_state = BOTTOM_LEFT;
            }
            break;
        case BOTTOM_LEFT:
            double distance_to_top_left = robot_position.dist_to(top_left_);
            if (check_is_done() || distance_to_top_left < 0.1) {
                next_state = TOP_LEFT;
            }
            break;
    }

    return next_state;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    WorldState* world_state = last_world_state_;

    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();

    switch (current_state_) {
        case TOP_LEFT:
            auto motion_instance = planning::LinearMotionInstant{robot_position, top_right_};
            auto motion_command = planning::MotionCommand{"path_target", motion_instance, planning::FaceAngle{0}, true};
            intent.motion_command = motion_command;
            break;
        case TOP_RIGHT:
            auto motion_instance = planning::LinearMotionInstant{robot_position, bottom_right_};
            auto motion_command = planning::MotionCommand{"path_target", motion_instance, planning::FaceAngle{0}, true};
            intent.motion_command = motion_command;
            break;
        case BOTTOM_RIGHT:
            auto motion_instance = planning::LinearMotionInstant{robot_position, bottom_left_};
            auto motion_command = planning::MotionCommand{"path_target", motion_instance, planning::FaceAngle{0}, true};
            intent.motion_command = motion_command;
            break;
        case BOTTOM_LEFT:
            auto motion_instance = planning::LinearMotionInstant{robot_position, top_left_};
            auto motion_command = planning::MotionCommand{"path_target", motion_instance, planning::FaceAngle{0}, true};
            intent.motion_command = motion_command;
            break;
    }
    return std::nullopt;
}
}  // namespace strategy
