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
            if (check_is_done()) {
                break;
            }
            if(current_play_state_ == PlayState::stop()) {
                return STOP;
            }
        }
        case STOP: {
            if(current_play_state_ != PlayState::stop()) {
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
            rj_geometry::Point target_pt{-1.35, 8.85};
            rj_geometry::Point target_vel{0.0, 0.0};

            // Create Motion Command
            planning::LinearMotionInstant goal{target_pt, target_vel};
            intent.motion_command =
                planning::MotionCommand{"path_target", goal, planning::FaceBall{}};
            return intent;
        }

        case STOP: {  // First, gets the robot to the ball to begin penalty dribbling-shooting

            intent.motion_command =
                planning::MotionCommand{"halt"};
            return intent;
        }
     
    }

    return intent;
}

std::string PenaltyPlayer::get_current_state() { return "PenaltyPlayer"; }

double PenaltyPlayer::distance_from_their_robots(rj_geometry::Point tail,
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
/**
 * @brief Iterates across 19 possible shot target locations along the goal width in 0.05-meter
 * increments. For each location, it calculates the clearance distance from opponent robots and
 * updates the best shot position if a better (less obstructed) option is found.
 * @return The best target position (farthest from obstacles) found after considering all 19
 * possibilities.
 */
rj_geometry::Point PenaltyPlayer::calculate_best_shot() const {
    // Goal location
    rj_geometry::Point their_goal_pos = field_dimensions_.their_goal_loc();
    // TODO: Consider reducing goal width variable to reduce the possibility of the shot missing at
    // edges
    double goal_width = field_dimensions_.goal_width();  // 1.0 meters

    // Ball location
    rj_geometry::Point ball_position = this->last_world_state_->ball.position;
    // Sets initial target shot at the middle of their goal
    rj_geometry::Point best_shot = their_goal_pos;
    double best_distance = -1.0;
    rj_geometry::Point increment(0.05, 0);
    rj_geometry::Point curr_point =
        their_goal_pos - rj_geometry::Point(goal_width / 2.0, 0) + increment;
    // Compare shots to find best possibility
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
