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
            // If we successfully rotate while the ball is still within possession 
            if (check_is_done()) { 
                return TRANSPORT;
            } //else if (distance_to_ball() > kRobotPossessionRadius) {
            //     return COLLECT;
            // }
            break;
        }
        case TRANSPORT: {
            // If we make it to the point 0.3 meters away, stop and go to STANDBY (the ball will roll to a stop at that distance)
            if (check_is_done()) {
                return STAND_BY;
            } else if (distance_to_ball() > kRobotPossessionRadius) { // If we lose possession, return to COLLECT
                return COLLECT;
            }
            break;
        }
        case STAND_BY: {
            // If the ball rolls 0.15m away from the designated spot, return to COLLECT
            if (ball_to_point_distance() > kBallPlacementDeadzoneRadius) {
                return COLLECT;
            }
            break;
        }
    }
    return latest_state_;
}

std::optional<RobotIntent> BallPlacer::state_to_task(RobotIntent intent) {
    switch (latest_state_) {
        // Runs "collect" command to go-to and grab the ball
        case COLLECT: { 
            SPDLOG_INFO("COLLECT");            
            
            auto pivot_cmd = planning::MotionCommand{"collect"};
            intent.motion_command = pivot_cmd;
            intent.dribbler_mode = RobotIntent::DribblerMode::ON;
            return intent;
        }
        // Rotates the ball to ensure we have possession
        case ROTATE: { 
            SPDLOG_INFO("ROTATE");

            planning::LinearMotionInstant target{current_play_state_.ball_placement_point().value()};
            auto pivot_cmd =
                planning::MotionCommand{"rotate", target, planning::FaceTarget{}, false};
            intent.motion_command = pivot_cmd;
            intent.dribbler_mode = RobotIntent::DribblerMode::ON;
            return intent;
        }
        // Runs a straight line to the ball_placement_point() given
        case TRANSPORT: {  
            SPDLOG_INFO("TRANSPORT");
            auto ballPlacement = current_play_state_.ball_placement_point();
            intent.motion_command = planning::MotionCommand{};
            if(ballPlacement.has_value()) {
                rj_geometry::Point robotToPoint =
                    (last_world_state_->get_robot(true, robot_id_).pose.position() -
                        ballPlacement.value());
                double slowDown = 2.0;
                double length = robotToPoint.mag() - kRobotRadius * slowDown;
                robotToPoint = -robotToPoint.normalized(length);
                planning::LinearMotionInstant target{
                    last_world_state_->get_robot(true, robot_id_).pose.position() + robotToPoint}; 
                
                intent.motion_command =
                    planning::MotionCommand{"path_target", target, planning::FacePoint{ballPlacement.value()}};
                intent.dribbler_mode = RobotIntent::DribblerMode::ON;

            } else {
                SPDLOG_ERROR("Ball position was not retrieved from PlayState");
            }
            return intent;
            break;
        }
        // Stops where it is
        case STAND_BY: {
            SPDLOG_INFO("STAND BY");
            intent.motion_command =
                planning::MotionCommand{"halt"}; // Maybe STANDBY not even needed?
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
