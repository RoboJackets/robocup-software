#include "testing.hpp"

namespace strategy {

Testing::Testing(int r_id) : Position(r_id), off_(r_id_) {
    position_name_ = "Testing";
    current_state_ = IDLING;
    move_on_ = false;
    r_id_ = counter++;
    is_running_.emplace_back(true);
}

std::optional<RobotIntent> Testing::derived_get_task(RobotIntent intent) {
    off_.set_globals(last_world_state_, field_dimensions_);
    current_state_ = update_state();
    SPDLOG_INFO(current_state_);
    // if (r_id_ == 1) {
    SPDLOG_INFO("Next says {}", next_);
    // }
    return state_to_task(intent);
}

Testing::State Testing::update_state() {
    State next_state = current_state_;
    WorldState* world_state = this->last_world_state_;
    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = world_state->ball.position;
    double distance_to_ball = robot_position.dist_to(ball_position);

    switch (current_state_) {
        case IDLING: {
            if (start() && next_ == NULLOPT) {
                next_state = BASIC_MOVEMENT_1;
            }
            if (next_ != NULLOPT && !lock_) {
                next_state = next_;
                lock_ = true;
            }
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
                next_ = BRING_TO_CENTER;
            }
            break;
        }

        case SPIN_1: {
            if (check_is_done()) {
                next_state = SPIN_2;
            }
            break;
        }

        case SPIN_2: {
            if (check_is_done()) {
                move_on_ = false;
                next_state = SPIN_1;
            }
        }

        case BRING_TO_CENTER: {
            is_running_.at(r_id_) = true;
            if (is_ball_near_center()) {
                if (!lock2_) {
                    next_state = LINEKICK;
                    off_.set_state_shooting();
                    lock_ = false;
                }
            }
            break;
        }

        case LINEKICK: {
            lock2_ = true;
            // if (derived_check() || distance_to_ball > ball_lost_distance_) {
            if (derived_check()) {
                lock2_ = false;
                move_on_ = false;
                next_state = IDLING;
                next_ = NULLOPT;
            }
            break;
        }

        default:
            next_state = IDLING;
    }

    return next_state;
}

std::optional<RobotIntent> Testing::state_to_task(RobotIntent intent) {
    WorldState* world_state = this->last_world_state_;
    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
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
            planning::PathTargetFaceOption face_option{
                planning::FacePoint{rj_geometry::Point{0, 4.5}}};
            auto motion_cmd = planning::MotionCommand{"path_target", target, face_option};
            intent.motion_command = motion_cmd;
            intent.is_active = true;
            break;
        }

        case SPIN_1: {
            planning::LinearMotionInstant target{robot_position};
            planning::PathTargetFaceOption face_option{
                planning::FacePoint{rj_geometry::Point{robot_position.y(), 0.0}}};
            auto motion_cmd = planning::MotionCommand{"path_target", target, face_option, true};
            intent.motion_command = motion_cmd;
            intent.is_active = true;
            break;
        }

        case SPIN_2: {
            planning::LinearMotionInstant target{robot_position};
            planning::PathTargetFaceOption face_option{
                planning::FacePoint{rj_geometry::Point{robot_position.x(), 9.0}}};
            auto motion_cmd = planning::MotionCommand{"path_target", target, face_option, true};
            intent.motion_command = motion_cmd;
            intent.is_active = true;
            break;
        }

        case BRING_TO_CENTER: {
            // wait
            break;
        }

        case LINEKICK: {
            return off_.derived_state_to_task(intent);
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

bool Testing::start() {
    is_running_.at(r_id_) = false;
    return (std::find(is_running_.begin(), is_running_.end(), true) == is_running_.end());
}

bool Testing::is_ball_near_center() {
    rj_geometry::Point ball_position = this->last_world_state_->ball.position;
    return (ball_position.dist_to(rj_geometry::Point(0, 4.5)) < 0.5);
}

bool Testing::derived_check() { return off_.derived_check_is_done(); }

}  // namespace strategy