#include "solo_offense.hpp"

namespace strategy {

SoloOffense::SoloOffense(const Position& other) : Position{other} {
    position_name_ = "SoloOffense";
}

SoloOffense::SoloOffense(int r_id) : Position{r_id, "SoloOffense"} {}

std::optional<RobotIntent> SoloOffense::derived_get_task(RobotIntent intent) {
    // Get next state, and if different, reset clock
    State new_state = next_state();
    if (new_state != current_state_) {
        reset_timeout();
        SPDLOG_INFO("SoloOffense ID {} is now {}", robot_id_, state_to_name(new_state));
    }
    current_state_ = new_state;

    // Calculate task based on state
    return state_to_task(intent);
}

std::string SoloOffense::get_current_state() {
    return std::string{"Solo Offense"} + std::to_string(static_cast<int>(current_state_));
}

SoloOffense::State SoloOffense::next_state() {
    // handle transitions between current state
    cached_ball_pos_ = get_ball_pos();

    if (point_in_red(get_ball_pos())) {
        return DEFAULT;
    }

    switch (current_state_) {
        case DEFAULT: {
            return TO_BALL;
        }
        case TO_BALL: {
            if (check_is_done()) {
                shot_target_ = field_dimensions_.their_goal_loc();
                return KICK;  // TODO: should we check if the ball is in front of us?
            }
        }
        case KICK: {
            if (check_is_done() || timed_out()) {
                return DEFAULT;
            }
        }
    }
    return current_state_;
}

std::optional<RobotIntent> SoloOffense::state_to_task(RobotIntent intent) {
    switch (current_state_) {
        case DEFAULT: {
            planning::MotionCommand afk{};
            intent.motion_command = afk;
            return intent;
        }
        case TO_BALL: {
            rj_geometry::Point ball_pos = get_ball_pos();
            rj_geometry::Point goal_pos = field_dimensions_.their_goal_loc();

            rj_geometry::Point shot_dir = (goal_pos - ball_pos).normalized();
            rj_geometry::Point shot_dot = ball_pos - shot_dir * kBackOffset;

            auto pivot_cmd =
                planning::MotionCommand{"path_target", planning::LinearMotionInstant{shot_dot},
                                        planning::FaceBall{}, false};

            intent.motion_command = pivot_cmd;

            return intent;
        }
        case KICK: {
            auto line_kick_cmd =
                planning::MotionCommand{"line_kick", planning::LinearMotionInstant{shot_target_}};

            intent.motion_command = line_kick_cmd;
            intent.dribbler_mode = RobotIntent::DribblerMode::OFF;
            intent.shoot_mode = RobotIntent::ShootMode::KICK;
            intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
            intent.kick_speed = 2.7;
            intent.is_active = true;

            return intent;
        }
    }
    return intent;
}

rj_geometry::Point SoloOffense::get_ball_pos() const {
    if (last_world_state_->ball.visible) {
        return last_world_state_->ball.position;
    } else {
        return cached_ball_pos_;
    }
}
bool SoloOffense::point_in_red(rj_geometry::Point concerned_point) const {
    return (field_dimensions_.our_defense_area().contains_point(concerned_point) ||
            field_dimensions_.their_defense_area().contains_point(concerned_point) ||
            !field_dimensions_.field_rect().contains_point(concerned_point));
}

rj_geometry::Point SoloOffense::calculate_best_shot() const {
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
double SoloOffense::distance_from_their_robots(rj_geometry::Point tail,
                                               rj_geometry::Point head) const {
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
}  // namespace strategy
