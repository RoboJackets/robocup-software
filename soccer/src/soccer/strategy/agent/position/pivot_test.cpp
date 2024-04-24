#include "pivot_test.hpp"

namespace strategy {

Pivot::Pivot(int r_id) : Position{r_id, "Pivot"} {}

std::optional<RobotIntent> Pivot::derived_get_task(RobotIntent intent) {
    // Get next state, and if different, reset clock
    State new_state = next_state();
    if (new_state != current_state_) {
        curr_pt_ = last_world_state_->get_robot(true, robot_id_).pose.position();
    }
    current_state_ = new_state;

    // Calculate task based on state
    return state_to_task(intent);
}

std::string Pivot::get_current_state() {
    return std::string{"Pivot"} + std::to_string(static_cast<int>(current_state_));
}

Pivot::State Pivot::next_state() {
    // handle transitions between current state
    switch (current_state_) {
        case OUR_GOAL: {
            if (check_is_done()) {
                count_++;
                return IDLE;
            }
        }
        case OPP_GOAL: {
            if (check_is_done()) {
                count_++;
                return IDLE;
            }
        }
        case IDLE: {
            if (current_play_state_.is_playing()) {
                if (count_ % 2 == 0) {
                    return OUR_GOAL;
                }
                return OPP_GOAL;
            }
        }
    }
    return current_state_;
}

std::optional<RobotIntent> Pivot::state_to_task(RobotIntent intent) {
    switch (current_state_) {
        case OUR_GOAL: {
            planning::LinearMotionInstant target{field_dimensions_.our_goal_loc()};
            auto pivot_cmd = planning::MotionCommand{"line_pivot", target, planning::FaceTarget{},
                                                     false, last_world_state_->ball.position};
            pivot_cmd.pivot_radius = 1;
            intent.motion_command = pivot_cmd;
            intent.dribbler_speed = 255.0;
            return intent;
        }
        case OPP_GOAL: {
            planning::LinearMotionInstant target{field_dimensions_.their_goal_loc()};
            auto pivot_cmd = planning::MotionCommand{"line_pivot", target, planning::FaceTarget{},
                                                     false, last_world_state_->ball.position};
            pivot_cmd.pivot_radius = 1;

            intent.motion_command = pivot_cmd;
            intent.dribbler_speed = 255.0;
            return intent;
        }
        case IDLE: {
            intent.motion_command = planning::MotionCommand{};
            return intent;
        }
    }
    return intent;
}
}  // namespace strategy
