#include "penalty_player.hpp"

namespace strategy {

PenaltyPlayer::PenaltyPlayer(int r_id) : Position(r_id, "PenaltyPlayer") {}

std::optional<RobotIntent> PenaltyPlayer::derived_get_task(RobotIntent intent) {
    latest_state_ = update_state();
    return state_to_task(intent);
}

PenaltyPlayer::State PenaltyPlayer::update_state() {
    switch (latest_state) {
        case LINE_UP: {
            // if penalty playing and restart penalty in playstate we switch to shooting
            if (current_play_state_.is_penalty_playing() && current_play_state.is_penalty()) {
                return SHOOTING_START;
            }
            break;
        }
        case SHOOTING_START: {
            if (check_is_done()) {
                return SHOOTING;
            }
            if (distance_to_ball() < kOwnBallRadius) {
                return SHOOTING;
            }
            break;
        }
        case SHOOTING: {
            if (check_is_done()) {
                return LINE_UP;
            }
            break;
        }
    }
    return latest_state_;
}

std::optional<RobotIntent> PenaltyPlayer::state_to_task(RobotIntent intent) {
    World State* world_state = last_world_state_;
    switch (latest_state_) {
        case LINE_UP: {
            rj_geometry::Point y_pos = world_state->ball.position.y();
            // if ball is above goal, increase y_pos, else decrease
            if (y_pos - field_dimensions.their_goal_loc() > 0) {
                y_pos += kRobotRadius - 0.15;
            } else {
                y_pos -= kRobotRadius - 0.15;
            }
            rj_geometry::Point target_pt{world_state->ball.position.x(), y_pos};
            rj_geometry::Point target_vel{0.0, 0.0};
            // Face ball
            planning::PathTargetFaceOption face_option{planning::FaceBall{}};
            // Avoid ball
            bool ignore_ball{false};

            // Create Motion Command
            planning::LinearMotionInstant goal{target_pt, target_vel};
            intent.motion_command = planning::MotionCommand{"path_target", goal, face_option, ignore_ball};
            break;
        }
        case SHOOTING_START: {
            target_ = calculate_best_shot();
            rj_geometry::Point ball_position = last_world_state_->ball.position;
            auto current_pos = last_world_state_->get_robot(true, robot_id_).pose.position();
            auto move_vector = (current_pos - ball_position).normalized(0.2);

            planning::LinearMotionInstant target{ball_position + move_vector};
            planning::MotionCommand prep_command{"path_target", target, planning::FaceBall{}};

            intent.motion_command = prep_command;

            return intent;
        }
        case SHOOTING: {
            auto line_kick_cmd =
                planning::MotionCommand{"line_kick", planning::LinearMotionInstant{target_}};

            intent.motion_command = line_kick_cmd;
            intent.shoot_mode = RobotIntent::ShootMode::KICK;
            intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
            intent.kick_speed = 4.0;

            return intent;
            break;
        }
    }

    return intent;
}

std::string PenaltyPlayer::get_current_state() { return "PenaltyPlayer"; }

double PenaltyPlayer::distance_from_their_robots(rj_geometry::Point tail, rj_geometry::Point head) const {
    rj_geometry::Point vec = head - tail;
    auto& their_robots = this->last_world_state_->their_robots;

    double min_angle = -0.5;
    for (auto enemy : their_robots) {
        rj_geometry::Point enemy_vec = enemy.pose.position() - tail;
        if (enemy_vec.dot(vec) < 0) {
            continue;
        }
        auto projection = (enemy_vec.dot(vec) / vec.dot(vec));
        enemy_vec = enemy_vec - (projection)*vec;
        double distance = enemy_vec.mag();
        if (distance < (kRobotRadius + kBallRadius)) {
            return -1.0;
        }
        double angle = distance / projection;
        if ((min_angle < 0) || (angle < min_angle)) {
            min_angle = angle;
        }
    }
    return min_angle;
}

rj_geometry::Point PenaltyPlayer::calculate_best_shot() const {
    // Goal location
    rj_geometry::Point their_goal_pos = field_dimensions_.their_goal_loc();
    double goal_width = field_dimensions_.goal_width();  // 1.0 meters

    // Ball location
    rj_geometry::Point ball_position = this->last_world_state_->ball.position;

    rj_geometry::Point best_shot = their_goal_pos;
    double best_distance = -1.0;
    rj_geometry::Point increment(0.05, 0);
    rj_geometry::Point curr_point =
        their_goal_pos - rj_geometry::Point(goal_width / 2.0, 0) + increment;
    for (int i = 0; i < 19; i++) {
        double distance = distance_from_their_robots(ball_position, curr_point);
        if (distance > best_distance) {
            best_distance = distance;
            best_shot = curr_point;
        }
        curr_point = curr_point + increment;
    }
    return best_shot;
}

void PenaltyPlayer::derived_acknowledge_pass() {}

void PenaltyPlayer::derived_pass_ball() {}

void PenaltyPlayer::derived_acknowledge_ball_in_transit() {}

}  // namespace strategy
