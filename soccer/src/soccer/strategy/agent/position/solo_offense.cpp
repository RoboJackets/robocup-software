#include "solo_offense.hpp"

namespace strategy {

SoloOffense::SoloOffense(const Position& other) : Position{other} {
    position_name_ = "SoloOffense";
}

SoloOffense::SoloOffense(int r_id) : Position{r_id, "SoloOffense"} {}

std::optional<RobotIntent> SoloOffense::derived_get_task(RobotIntent intent) {
    // Get next state.
    State new_state = DEFAULT;
    if (!timed_out()) {  // on timeout, move to DEFAULT
        new_state = next_state();
    }

    // Check if a transition occurred.
    if (new_state != current_state_) {
        SPDLOG_INFO("Robot {}: Transitioned from {} -> {}", robot_id_,
                    state_to_name(current_state_), state_to_name(new_state));
        reset_timeout();
    }
    current_state_ = new_state;

    // Calculate task based on state
    return state_to_task(intent);
}

/**
 * @brief A richer to_string.
 */
std::string SoloOffense::get_current_state() {
    return std::string("SoloOffense-") +
           std::string(state_to_name(current_state_));  // SoloOffense-MARKER, e.g.
}

/**
 * @brief State transitions.
 */
SoloOffense::State SoloOffense::next_state() {
    rj_geometry::Point ball_pose = last_world_state_->ball.position;
    rj_geometry::Point our_pose = last_world_state_->get_robot(true, robot_id_).pose.position();
    double our_dist = our_pose.dist_to(ball_pose);

    // Scan opponent positions to determine transition.
    double closest_dist = std::numeric_limits<double>::infinity();
    for (int i = 0; i < 6; i++) {  // TODO: replace 6 with number of opponent robots (to be proper)
        RobotState opp = last_world_state_->get_robot(false, i);
        rj_geometry::Point opp_pose = opp.pose.position();
        double opp_dist = opp_pose.dist_to(ball_pose);  // distance from opp to ball
        if (opp_dist < closest_dist) {
            marking_id_ = i;
            closest_dist = opp_dist;
        }
    }
    // marking_id_ contains the shell id of the opponent robot closest to the ball
    // closest_dist contains the distance from the robot to the ball

    // UNCONDITIONAL TRANSITIONS
    if (!field_dimensions_.field_coordinates().contains_point(ball_pose)) {  // Ball OOB.
        return DEFAULT;
    }
    if ((our_dist > closest_dist) &&
        (closest_dist < kPosessionRadius)) {  // Opponent vaguely has possession.
        return MARKER;
    }
    // CONDITIONAL TRANSITIONS
    switch (current_state_) {
        case DEFAULT:
            return SHOOTING_START;
        case MARKER: {
            return SHOOTING_START;
        }
        case SHOOTING_START: {
            if (check_is_done()) {
                return SHOOTING_PIVOT;
            }  // ball is collected
            return SHOOTING_START;
        }
        case SHOOTING_PIVOT: {
            if (check_is_done()) {
                return SHOOTING_KICK;
            }  // is facing target
            return SHOOTING_PIVOT;
        }
        case SHOOTING_KICK: {
            if (our_pose.dist_to(ball_pose) > kPosessionRadius) {
                return DEFAULT;
            }  // ball is kicked (or we've been robbed)
            return SHOOTING_KICK;
        }
    }
    return current_state_;
}

/**
 * @brief State behaviors.
 */
std::optional<RobotIntent> SoloOffense::state_to_task(RobotIntent intent) {
    switch (current_state_) {
        case DEFAULT: {
            auto cmd = planning::MotionCommand{"halt"};

            intent.motion_command = cmd;
            return intent;
        }
        case MARKER: {
            auto mark_pose = last_world_state_->get_robot(false, marking_id_).pose.position();
            auto mark_defend_pose =
                mark_pose +
                (field_dimensions_.our_goal_loc() - mark_pose).normalized(kMarkDistance);
            // Move in between the mark and the goal.

            auto cmd = planning::MotionCommand{"path_target",
                                               planning::LinearMotionInstant{mark_defend_pose},
                                               planning::FaceBall{}, true};

            intent.motion_command = cmd;
            return intent;
        }
        case SHOOTING_START: {
            auto cmd = planning::MotionCommand{"collect"};

            intent.motion_command = cmd;
            intent.dribbler_speed = 255;
            return intent;
        }
        case SHOOTING_PIVOT: {
            shot_target_ = calculate_best_shot();

            auto cmd =
                planning::MotionCommand{"rotate", planning::LinearMotionInstant{shot_target_},
                                        planning::FaceTarget{}, false};

            intent.motion_command = cmd;
            intent.dribbler_speed = 255;

            return intent;
        }
        case SHOOTING_KICK: {
            auto cmd = planning::MotionCommand{
                "path_target",
                planning::LinearMotionInstant{shot_target_},  // target obtained from pivot
                planning::FaceTarget{}, true};

            intent.motion_command = cmd;
            intent.dribbler_speed = 255;
            intent.shoot_mode = RobotIntent::ShootMode::KICK;
            intent.trigger_mode = RobotIntent::TriggerMode::IMMEDIATE;
            intent.kick_speed = 4.0;  // TODO: what's the best value for this?

            return intent;
        }
    }
    return intent;
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
