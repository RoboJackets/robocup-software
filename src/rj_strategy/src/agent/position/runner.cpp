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
            case FORWARD: {
                if (y > 10) {
                    return LEFT;
                }
                return current_state_;
            }
            case LEFT: {
                if (x < -10) {
                    return BACKWARD;
                }
                return current_state_;
            }
            case BACKWARD: {
                if (y < -10) {
                    return RIGHT;
                } else {
                    return BACKWARD;
                }
                return current_state_;
            }
            case RIGHT: {
                if (x > 10) {
                    return FORWARD;
                }
                return current_state_;
            }
            return current_state_;
        }
    }
}