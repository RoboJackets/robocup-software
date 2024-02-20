#include "testing.hpp"

namespace strategy {

Testing::Testing(int r_id) : Position(r_id) {
    position_name_ = "Testing";
    current_state_ = IDLING;
    move_on_ = false;
    r_id_ = r_id;
    is_running_.emplace_back(true);
}

std::optional<RobotIntent> Testing::derived_get_task(RobotIntent intent) {
    current_state_ = update_state();
    return state_to_task(intent);
}

Testing::State Testing::update_state() {
    State next_state = current_state_;

    switch (current_state_) {
        case IDLING: {
            next_state = BASIC_MOVEMENT_1;
            break;
        }
            
        case BASIC_MOVEMENT_1: {
            if (proceed()) {
                move_on_ = false;
                next_state = BASIC_MOVEMENT_2;
            }
            break;
        }

        case BASIC_MOVEMENT_2: {
            if (proceed()) {
                move_on_ = false;
                next_state = IDLING;
            }
            break;
        }

        default:
            next_state = IDLING;

    }

    return next_state;
}

std::optional<RobotIntent> Testing::state_to_task(RobotIntent intent) {
    switch (current_state_) {

        case IDLING: {
            break;
        }

        case BASIC_MOVEMENT_1: {
            rj_geometry::Point target_pos = rj_geometry::Point(-.24 + .6 * robot_id_, 1.75);
            planning::LinearMotionInstant target{target_pos};
            auto motion_cmd = planning::MotionCommand{"path_target", target};
            intent.motion_command = motion_cmd;
            intent.is_active = true;
            break;
        }
        
        case BASIC_MOVEMENT_2: {
            rj_geometry::Point target_pos = rj_geometry::Point(-.24 + .6 * robot_id_, 7.0);
            planning::LinearMotionInstant target{target_pos};
            auto motion_cmd = planning::MotionCommand{"path_target", target};
            intent.motion_command = motion_cmd;
            intent.is_active = true;
            break; 
        }

    }

    return intent;
}

bool Testing::proceed() {
    bool done = check_is_done();
    if (done || move_on_) {
        is_running_.at(r_id_) = false;
        move_on_ = true;
    } else {
        is_running_.at(r_id_) = true;
    }
    return (std::find(is_running_.begin(), is_running_.end(), true) == is_running_.end());
}


} // namespace strategy