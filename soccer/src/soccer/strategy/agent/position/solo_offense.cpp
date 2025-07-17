#include "solo_offense.hpp"
#include <random>

namespace {
/// Thread‑local PRNG seeded once per thread.
inline std::mt19937& rng() {
    // `std::random_device` is usually sufficient for seeding here.
    static thread_local std::mt19937 gen{std::random_device{}()};
    return gen;
}

/// Uniform real in (low, high).  End‑points are *open* if low/high are finite.
inline double uniform_real(double low, double high) {
    std::uniform_real_distribution<double> dist(low, high);
    return dist(rng());
}

/// Uniformly returns ‑1 or +1.
inline int random_sign() {
    // Bernoulli is a bit clearer for a two‑way choice.
    static thread_local std::bernoulli_distribution flip(0.5);
    return flip(rng()) ? 1 : -1;
}
}  // namespace
// Random my beloved

namespace strategy {

SoloOffense::SoloOffense(const Position& other) : Position{other} {
    position_name_ = "SoloOffense";
}

SoloOffense::SoloOffense(int r_id) : Position{r_id, "SoloOffense"} {}

std::optional<RobotIntent> SoloOffense::derived_get_task(RobotIntent intent) {
    // Cache ball position
    cached_ball_pos_ = get_ball_pos();

    // Get next state, and if different, reset clock
    State new_state = next_state();
    if (current_state_ != new_state) {
        reset_timeout();
        SPDLOG_INFO("Robot {}: {} -> {}", robot_id_, std::to_string(static_cast<int>(current_state_)), std::to_string(static_cast<int>(new_state)));
    }
    current_state_ = new_state;

    // Calculate task based on state
    return state_to_task(intent);
}

std::string SoloOffense::get_current_state() {
    return std::string{"Solo Offense"} + std::to_string(static_cast<int>(current_state_));
}

SoloOffense::State SoloOffense::next_state() {
    if (teammate_attacking()) { // Short-circuit: if a friendly has possession, chill out
        return DEFAULT; // TODO: replace this with the seeker sub-position instead of doing nothing
    }
    marking_id_ = find_mark();
    if (marking_id_ != -1) { // Short-circuit: if an enemy has possession, mark them
        return MARKER;
    }

    // Otherwise, follow the state machine:
    switch (current_state_) {
        case DEFAULT: {
            return TO_BALL;
        }
        case MARKER: {
            return TO_BALL;
        }
        case TO_BALL: {
            if (check_is_done()) {
                gather_target_ = calculate_gather();
                if (!point_in_red(gather_target_)) {
                    return GATHER_STEP;
                }
                juke_target_ = calculate_juke();
                if (!point_in_red(juke_target_)) {
                    return SIDE_STEP;
                }
                shot_target_ = calculate_best_shot();
                return AIM_AND_SHOOT;
            }
            return TO_BALL;
        }
        case GATHER_STEP: {
            if (check_is_done() || timed_out()) {
                if (!ball_in_dribbler()) { // we *should* have the ball
                    return DEFAULT;
                }
                
                juke_target_ = calculate_juke();
                if (!point_in_red(juke_target_)) {
                    return SIDE_STEP;
                }
                shot_target_ = calculate_best_shot();
                return AIM_AND_SHOOT;
            }
            return GATHER_STEP;
        }
        case SIDE_STEP: {
            if (check_is_done() || timed_out()) {
                if (!ball_in_dribbler()) { // we *should* have the ball
                    return DEFAULT;
                }
    
                shot_target_ = calculate_best_shot();
                return AIM_AND_SHOOT;
            }
            return SIDE_STEP;
        }
        case AIM_AND_SHOOT: {
            if (check_is_done()) {
                return DEFAULT;
            }
            return AIM_AND_SHOOT;
        }
    }
    return current_state_;
}

std::optional<RobotIntent> SoloOffense::state_to_task(RobotIntent intent) {
    switch (current_state_) {
        case DEFAULT: {
            return std::nullopt;
        }
        case MARKER: {
            if (marking_id_ == -1) { return std::nullopt; } // guard against weird races that cause index OOB crashes
            auto marker_target_pos =
                last_world_state_->get_robot(false, marking_id_).pose.position();
            auto target =
                marker_target_pos +
                (field_dimensions_.our_goal_loc() - marker_target_pos).normalized(kRobotRadius * 5);
            auto mark_cmd = planning::MotionCommand{
                "path_target", planning::LinearMotionInstant{target}, planning::FaceBall{}, true};
            intent.motion_command = mark_cmd;

            return intent;
        }
        case TO_BALL: {
            rj_geometry::Point robotToBall =
                (last_world_state_->ball.position -
                 last_world_state_->get_robot(true, robot_id_).pose.position());
            double slowDown = 1.0;
            double length = robotToBall.mag() - kRobotRadius * slowDown;
            robotToBall = robotToBall.normalized(length);
            planning::LinearMotionInstant target{
                last_world_state_->get_robot(true, robot_id_).pose.position() + robotToBall};
            auto pivot_cmd = planning::MotionCommand{"collect"};
            intent.motion_command = pivot_cmd;

            return intent;
        }
        case GATHER_STEP: {
            auto mark_cmd = planning::MotionCommand{"path_target", planning::LinearMotionInstant{gather_target_}, planning::FaceBall{}, true}; // TODO: try to set a slower velocity on the gather step by adding a default value to the path_target command that overrides max velocity
            intent.motion_command = mark_cmd;
            intent.dribbler_mode = RobotIntent::DribblerMode::ON;

            return intent;
        }
        case SIDE_STEP: {
            // rotate to some direction vaguely facing the goal
            auto juke_cmd = planning::MotionCommand{"path_target", planning::LinearMotionInstant{juke_target_}, planning::FaceTarget{}, true}; // TODO: facing the target will not suffice, we need to face in a particular angle to make sure dribbling sticks irl
            intent.motion_command = juke_cmd;
            intent.dribbler_mode = RobotIntent::DribblerMode::ON;

            return intent;
        }
        case AIM_AND_SHOOT: {
            auto pivot_cmd = planning::MotionCommand{"rotate", planning::LinearMotionInstant{shot_target_}, planning::FaceTarget{}, false};
            intent.motion_command = pivot_cmd;
            intent.dribbler_mode = RobotIntent::DribblerMode::ON;
            intent.trigger_mode = RobotIntent::TriggerMode::AT_END;
            intent.kick_speed = 4.0;
            return intent;
        }
    }
    return intent;
}

rj_geometry::Point SoloOffense::calculate_gather() const {
    // move a little bit forward to snag the ball in the dribbler, in case collect fumbles it
    rj_geometry::Pose robot_pose = last_world_state_->get_robot(true, robot_id_).pose;
    return robot_pose.position() + rj_geometry::Point{
        kGatherLength * std::cos(robot_pose.heading()),
        kGatherLength * std::sin(robot_pose.heading())
    };
}

rj_geometry::Point SoloOffense::calculate_juke() const {
    // rotate to some direction vaguely facing the goal
    rj_geometry::Point robo = last_world_state_->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point gol = calculate_best_shot();
    rj_geometry::Point shot_direction = (gol - robo).normalized();
    rj_geometry::Point perp_direction(-shot_direction.y(), shot_direction.x());

    double side_step_dist = uniform_real(0.3, 0.8);
    double left_or_right  = static_cast<double>(random_sign());

    rj_geometry::Point forward_offset = shot_direction * 0.1;
    rj_geometry::Point lateral_offset = perp_direction * left_or_right * side_step_dist;

    return robo + forward_offset + lateral_offset;
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

bool SoloOffense::point_in_red(rj_geometry::Point concerned_point) const {
    return (field_dimensions_.our_defense_area().contains_point(concerned_point) ||
            field_dimensions_.their_defense_area().contains_point(concerned_point) ||
            !field_dimensions_.field_rect().contains_point(concerned_point));
}

rj_geometry::Point SoloOffense::get_ball_pos() const {
    if (last_world_state_->ball.visible) {
        return last_world_state_->ball.position;
    } else {
        return cached_ball_pos_;
    }
}

bool SoloOffense::ball_in_dribbler() const {
    if (!last_world_state_->ball.visible) {
        return true;
    }
    rj_geometry::Pose robo_pose = last_world_state_->get_robot(true, robot_id_).pose;
    rj_geometry::Point dribbler_pos = robo_pose.position() + rj_geometry::Point::direction(robo_pose.heading()) * kRobotMouthRadius;
    // Check if ball is within tolerance of dribbler target
    return (get_ball_pos() - dribbler_pos).mag() <= kDribblerTolerance;
}

bool SoloOffense::teammate_attacking() const {
    // Don't go for an attack if a teammate has possession.
    float my_dist = last_world_state_->get_robot(true, robot_id_).pose.position().dist_to(get_ball_pos());
    for (int i = 0; i < kNumShells; i++) {
        if (i == robot_id_) { continue; } // obviously, don't consider yourself in this check
        RobotState teamy = last_world_state_->get_robot(true, i);
        if (!teamy.visible) { continue; }
        rj_geometry::Point teamy_pos = teamy.pose.position();
        float curr_dist = teamy_pos.dist_to(get_ball_pos());
        if (curr_dist < kWideRobotTolerance && curr_dist < my_dist) {
            return true; // a teammate has it handled, stand down
        }
    }
    return false; // ur good to go
}

int SoloOffense::find_mark() const {
    int candidate = 0;
    double closest_dist = std::numeric_limits<double>::infinity();
    for (int i = 0; i < kNumShells; i++) {
        RobotState opp = last_world_state_->get_robot(false, i);
        if (!opp.visible) { continue; }
        rj_geometry::Point opp_pos = opp.pose.position();
        if (point_in_red(opp_pos)) { continue; } // do NOT mark people in red zones, since the the marking intent will ignore that and edge the red zone
        float curr_dist = opp_pos.dist_to(get_ball_pos());
        if (curr_dist < closest_dist) { // TODO: if there are 2 solo offenses, they will mark the same opponent; this is probably bad
            candidate = i;
            closest_dist = curr_dist;
        }
    }

    if (closest_dist < kWideRobotTolerance || point_in_red(get_ball_pos())) {
        return candidate;
    } else {
        return -1;
    }
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
