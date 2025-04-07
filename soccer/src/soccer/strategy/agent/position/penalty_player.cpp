#include "penalty_player.hpp"

namespace strategy {

// ALSO USED AS KickoffKicker
PenaltyPlayer::PenaltyPlayer(int r_id) : Position(r_id, "PenaltyPlayer") {}

PenaltyPlayer::PenaltyPlayer(const Position& other) : Position{other} {}

std::optional<RobotIntent> PenaltyPlayer::derived_get_task(RobotIntent intent) {
    latest_state_ = update_state();
    return state_to_task(intent);
}

PenaltyPlayer::State PenaltyPlayer::update_state() {
    switch (latest_state_) {
        case START: {
            // if penalty playing and restart penalty in playstate we switch to shooting
            if (current_play_state_.is_ready() &&
                (current_play_state_.is_penalty() || current_play_state_.is_kickoff())) {
                return SMALL_KICK;
            }
            break;
        }
        case SMALL_KICK: {
            if (distance_from_enemy_goal() < kDistanceToGoalThreshold) {
                return LINE_UP;
            }
            break;
        }
        case LINE_UP: {
            if (check_is_done() || distance_to_ball() < kOwnBallRadius) {
                return SHOOTING_START;
            }
            break;
        }
        case SHOOTING_START: {
            if (check_is_done()) {
                return SHOOTING;
            }

            break;
        }
        case SHOOTING: {
            if (check_is_done()) {
                return START;
            }
            break;
        }
    }
    return latest_state_;
}

std::optional<RobotIntent> PenaltyPlayer::state_to_task(RobotIntent intent) {
    switch (latest_state_) {
        case START: {  // First, gets the robot to the ball to begin penalty dribbling-shooting
            double y_pos = last_world_state_->ball.position.y();
            // Add 0.3 buffer space to the y_pos of the ball to ensure the robot does not
            // hit the ball before being properly lined up behind it
            y_pos -= kRobotRadius + 0.3;
            rj_geometry::Point target_pt{last_world_state_->ball.position.x(), y_pos};
            rj_geometry::Point target_vel{0.0, 0.0};
            // Face ball
            planning::PathTargetFaceOption face_option{planning::FaceBall{}};

            // Create Motion Command
            planning::LinearMotionInstant goal{target_pt, target_vel};
            intent.motion_command =
                planning::MotionCommand{"path_target", goal, planning::FaceBall{}};
            return intent;
        }
        case SMALL_KICK: {  // less of a kick, more of a "follow" ball closely
            rj_geometry::Point center_goal = field_dimensions_.their_goal_loc();
            auto line_kick_cmd =
                planning::MotionCommand{"line_kick", planning::LinearMotionInstant{center_goal}};

            intent.motion_command = line_kick_cmd;
            intent.shoot_mode = RobotIntent::ShootMode::KICK;
            intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
            intent.kick_speed = 0.0;
            // The point of making a 0 kick speed is to fake dribble since we cannot get the vaccum
            // behavior to work

            return intent;
        }
        case LINE_UP: {  // Gets the robot behind the ball with a certain distance (usually
                         // immediatly skipped if the robot is already close)
            double y_pos = last_world_state_->ball.position.y();
            y_pos -= kOwnBallRadius;
            rj_geometry::Point target_pt{last_world_state_->ball.position.x(), y_pos};
            rj_geometry::Point target_vel{0.0, 0.0};
            // Face ball
            planning::PathTargetFaceOption face_option{planning::FaceBall{}};
            // Avoid ball
            bool ignore_ball{false};

            // Create Motion Command
            planning::LinearMotionInstant goal{target_pt, target_vel};
            intent.motion_command =
                planning::MotionCommand{"path_target", goal, face_option, ignore_ball};

            return intent;
        }
        case SHOOTING_START: {  // Positions the robot behind the ball at a certain angle so that it
                                // has a straight shot towards the goal
            target_ = calculate_best_shot();
            rj_geometry::Point ball_position = last_world_state_->ball.position;
            auto current_pos = last_world_state_->get_robot(true, robot_id_).pose.position();
            auto move_vector = (current_pos - ball_position).normalized(0.2);

            planning::LinearMotionInstant target{ball_position + move_vector};
            planning::MotionCommand prep_command{"path_target", target, planning::FaceBall{}};

            intent.motion_command = prep_command;

            return intent;
        }
        case SHOOTING: {  // Kicks the ball with a now much higher speed (basically SMALL_KICK but
                          // power set to 4)
            auto line_kick_cmd =
                planning::MotionCommand{"line_kick", planning::LinearMotionInstant{target_}};

            intent.motion_command = line_kick_cmd;
            intent.shoot_mode = RobotIntent::ShootMode::KICK;
            intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
            intent.kick_speed = 4.0;

            return intent;
        }
    }

    return intent;
}

std::string PenaltyPlayer::get_current_state() { return "PenaltyPlayer"; }

void PenaltyPlayer::derived_acknowledge_pass() {}

void PenaltyPlayer::derived_pass_ball() {}

void PenaltyPlayer::derived_acknowledge_ball_in_transit() {}

}  // namespace strategy
