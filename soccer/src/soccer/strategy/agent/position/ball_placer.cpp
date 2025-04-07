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
            if (check_is_done()) {
                return ROTATE;
            }
            break;
        }
        case ROTATE: {
            if (check_is_done()) {
                return TRANSPORT;
            }
            break;
        }
        case TRANSPORT: {
            if (check_is_done()) {
                return STAND_BY;
            }
            break;
        }
        case STAND_BY: {
            if (ball_to_point_distance() > 0.15) {
                return COLLECT;
            }
            break;
        }
    }
    return latest_state_;
}

std::optional<RobotIntent> BallPlacer::state_to_task(RobotIntent intent) {
    switch (latest_state_) {
        case COLLECT: { // MAKE SURE TO REMOVE GOAL KEEPER OBSTACLE!!!
            SPDLOG_INFO("COLLECT");
            // issue here relates to obstacle making within collect
            // the ball has an obstacle around it when it's in STOP playstate
            // go to plan_request.cpp check if(in.min_dist_from_ball....)
            auto collect_cmd = planning::MotionCommand{"collect"};
            intent.motion_command = collect_cmd;
            intent.dribbler_mode = RobotIntent::DribblerMode::ON;
            return intent;
        }
        case ROTATE: { // Phase causes immediate crash of simulator, suspicious of target setting
            SPDLOG_INFO("ROTATE");
            rj_geometry::Point target_vel{0.0, 0.0};
            planning::LinearMotionInstant target{current_play_state_.ball_placement_point().value()};
            auto pivot_cmd =
                planning::MotionCommand{"rotate", target, planning::FaceTarget{}, false};
            intent.motion_command = pivot_cmd;
            intent.dribbler_mode = RobotIntent::DribblerMode::ON;
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
                intent.dribbler_mode = RobotIntent::DribblerMode::ON;
            } else {
                SPDLOG_ERROR("Ball position was not retrieved from PlayState");
            }
            return intent;
            break;
        }
        case STAND_BY: {
            SPDLOG_INFO("STAND BY");
            double y_pos = last_world_state_->ball.position.y();
            // Add 0.3 buffer space to the y_pos of the ball to ensure the robot does not
            // hit the ball before being properly lined up behind it
            y_pos -= kRobotRadius + 0.2;
            rj_geometry::Point target_pt{last_world_state_->ball.position.x(), y_pos};
            rj_geometry::Point target_vel{0.0, 0.0};
            planning::PathTargetFaceOption face_option{planning::FaceBall{}};

            // Create Motion Command
            planning::LinearMotionInstant goal{target_pt, target_vel};
            intent.motion_command =
                planning::MotionCommand{"path_target", goal, planning::FaceBall{}};
            return intent;
            break;
        }
    }

    return intent;
}

std::string BallPlacer::get_current_state() { return "BallPlacer"; }

void BallPlacer::derived_acknowledge_pass() {}

void BallPlacer::derived_pass_ball() {}

void BallPlacer::derived_acknowledge_ball_in_transit() {}

}  // namespace strategy
