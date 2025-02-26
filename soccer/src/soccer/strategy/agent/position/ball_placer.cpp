#include "ball_placer.hpp"

namespace strategy {

// ALSO USED AS KickoffKicker
Ball_Placer::Ball_Placer(int r_id) : Position(r_id, "Ball_Placer") {}

Ball_Placer::Ball_Placer(const Position& other) : Position{other} {}

std::optional<RobotIntent> Ball_Placer::derived_get_task(RobotIntent intent) {
    latest_state_ = update_state();
    return state_to_task(intent);
}

Ball_Placer::State Ball_Placer::update_state() {
    switch (latest_state_) {
        case COLLECT: {
            if (check_is_done()) {
                return TRANSPORT;
            }
            break;
        }
        case TRANSPORT: {
            if (check_is_done()) {
                return COLLECT;
            }
            break;
        }
    }
    return latest_state_;
}

std::optional<RobotIntent> Ball_Placer::state_to_task(RobotIntent intent) {
    switch (latest_state_) {
        case COLLECT: {  
            intent.motion_command =
                planning::MotionCommand{"collect"};
            intent.dribbler_speed = 255.0;
            return intent;
        }
        case TRANSPORT: {  
            planning::LinearMotionInstant goal{target_pt, target_vel};
            intent.motion_command =
                planning::MotionCommand{"path_target", ball_placement_point_, face_option, ignore_ball};
            intent.dribbler_speed = 255.0;
            return intent;
        }
    }

    return intent;
}

std::string Ball_Placer::get_current_state() { return "Ball_Placer"; }

void Ball_Placer::derived_acknowledge_pass() {}

void Ball_Placer::derived_pass_ball() {}

void Ball_Placer::derived_acknowledge_ball_in_transit() {}

}  // namespace strategy
