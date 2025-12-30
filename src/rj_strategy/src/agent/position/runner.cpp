#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {
    Runner::Runner(int r_id) : Position{r_id, "Runner"}{}

    Runner::Runner(Position&& other) : Position{std::move(other)} {
        position_name_ = "Runner";
    }
    Runner::State Runner::next_state() {
        rj_geometry::Point curr_pos = last_world_state_->get_robot(true, robot_id_).pose.position();

        const double dist_threshold = .15;
        // handle transitions between current state
        switch (current_state_) {
            case UP: {
                if (curr_pos.dist_to(top_right) < dist_threshold) {
                    return LEFT;
                }
                return current_state_;
            }
            case LEFT: {
                if (curr_pos.dist_to(top_left) < dist_threshold) {
                    return DOWN;
                }
                return current_state_;
            }
            case DOWN: {
                if (curr_pos.dist_to(bottom_left) < dist_threshold) {
                    return RIGHT;
                }
                return current_state_;
            }
            case RIGHT: {
                if (curr_pos.dist_to(bottom_right) < dist_threshold) {
                    return UP;
                }
                return current_state_;
            }
            return current_state_;
        }
    }

    std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
        // Get next state, and if different, reset clock
        State new_state = next_state();
        if (current_state_ != new_state) {
            SPDLOG_INFO("Robot {}: now {}", robot_id_, get_current_state());
        }
        current_state_ = new_state;
        // Calculate task based on state
        return state_to_task(intent);
    }

    std::string Runner::get_current_state() {
        return std::string{"Runner"} + std::to_string(static_cast<int>(current_state_));
    }

    std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
        switch (current_state_) {
            case UP: {
                // Head towards top right corner
                rj_geometry::Point target_pos{Runner::top_right};
                planning::LinearMotionInstant move_to_point{target_pos};
                intent.motion_command = planning::MotionCommand{"path_target", move_to_point, 
                    planning::FacePoint{target_pos}, true};
                return intent;
            }
            case DOWN: {
                // Head towards bottom left corner
                rj_geometry::Point target_pos{Runner::bottom_left};
                planning::LinearMotionInstant move_to_point{target_pos};
                intent.motion_command = planning::MotionCommand{"path_target", move_to_point, 
                    planning::FacePoint{target_pos}, true};
                return intent;
            }
            case RIGHT: {
                // Head towards bottom right corner
                rj_geometry::Point target_pos{Runner::bottom_right};
                planning::LinearMotionInstant move_to_point{target_pos};
                intent.motion_command = planning::MotionCommand{"path_target", move_to_point, 
                    planning::FacePoint{target_pos}, true};
                return intent;
            }
            case LEFT: {
                // Head towards top left corner
                rj_geometry::Point target_pos{Runner::top_left};
                planning::LinearMotionInstant move_to_point{target_pos};
                intent.motion_command = planning::MotionCommand{"path_target", move_to_point, 
                    planning::FacePoint{target_pos}, true};
                return intent;
            }
    }
    }

}