#include "ball_placer.hpp"

namespace strategy {

// ALSO USED AS KickoffKicker
BallPlacer::BallPlacer(int r_id) : Position(r_id, "BallPlacer") {}

BallPlacer::BallPlacer(const Position& other) : Position{other} {}

std::optional<RobotIntent> BallPlacer::derived_get_task(RobotIntent intent) {
    latest_state_ = update_state();
    return state_to_task(intent);
}

BallPlacer::State BallPlacer::update_state() {
    switch (latest_state_) {
        case COLLECT: {
            if (distance_to_ball() < kOwnBallRadius+0.1) {
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

std::optional<RobotIntent> BallPlacer::state_to_task(RobotIntent intent) {
    switch (latest_state_) {
        case COLLECT: { 
            auto collect_cmd = planning::MotionCommand{"collect"};
            intent.motion_command = collect_cmd;
            intent.dribbler_speed = 255.0;
            return intent;
        }
        case TRANSPORT: {  
            SPDLOG_INFO("TRANSPORT");
            auto ballPlacement = current_play_state_.ball_placement_point();
            intent.motion_command = planning::MotionCommand{};
            if(ballPlacement.has_value()) {
                rj_geometry::Point target_vel{0.0, 0.0};
                planning::LinearMotionInstant target{ballPlacement.value(), target_vel};
                
                intent.motion_command =
                    planning::MotionCommand{"path_target", target,planning::FaceBall{}};
                intent.dribbler_speed = 255.0;
            } else {
                SPDLOG_ERROR("Ball position was not retrieved from PlayState");
            }
            return intent;
        }
    }

    return intent;
}

std::string BallPlacer::get_current_state() { return "BallPlacer"; }

void BallPlacer::derived_acknowledge_pass() {}

void BallPlacer::derived_pass_ball() {}

void BallPlacer::derived_acknowledge_ball_in_transit() {}

}  // namespace strategy
