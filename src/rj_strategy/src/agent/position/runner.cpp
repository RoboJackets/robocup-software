#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {
    Runner::Runner(int r_id) : Position{r_id, "Runner"}{}

    Runner::Runner(const Position& other) : Position{other} {
        position_name_ = "Runner";
    }

    Runner::State Runner::next_state() {
        double x = last_world_state->get_robot(true, robot_id_).pose.position().x_();
        double y = last_world_state->get_robot(true, robot_id_).pose.position().y_();
        // handle transitions between current state
        switch (current_state_) {
            case DEFAULT: {
                return FORWARD;
            }
            case UP: {
                if (y < .5) {
                    return LEFT;
                }
                return current_state_;
            }
            case LEFT: {
                if (x > 2.5) {
                    return DOWN;
                }
                return current_state_;
            }
            case DOWN: {
                if (y > 8.5) {
                    return RIGHT;
                }
                return current_state_;
            }
            case RIGHT: {
                if (x < -2.5) {
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
            SPDLOG_INFO("Robot {}: now {}", robot_id_, state_to_name(current_state_));
        }
        current_state_ = new_state;
        // Calculate task based on state
        return state_to_task(intent);
    }

    std::string Runner::get_current_state() {
        return std::string{"Runner"} + std::to_string(static_cast<int>(current_state_));
    }

}