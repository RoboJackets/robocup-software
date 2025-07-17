#include "free_kicker.hpp"

namespace strategy {

FreeKicker::FreeKicker(int r_id) : Position(r_id, "FreeKicker") {}

FreeKicker::FreeKicker(const Position& other) : Position{other} { position_name_ = "FreeKicker"; }

rj_geometry::Point FreeKicker::calculate_best_shot() const {
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
double FreeKicker::distance_from_their_robots(rj_geometry::Point tail, rj_geometry::Point head) const {
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


std::optional<RobotIntent> FreeKicker::derived_get_task(RobotIntent intent) {
    // FreeKicker is used for every set piece (kickoff, free, penalty)
    auto shot_target = calculate_best_shot();
    auto kick_cmd = planning::MotionCommand{"line_kick", planning::LinearMotionInstant{shot_target}};

    intent.motion_command = line_kick_cmd;
    intent.shoot_mode = RobotIntent::ShootMode::KICK;
    intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
    intent.kick_speed = 4.0;
    intent.is_active = true;

    return intent;
}

std::string FreeKicker::get_current_state() { return "FreeKicker"; }

void FreeKicker::derived_acknowledge_pass() {}

void FreeKicker::derived_pass_ball() {}

void FreeKicker::derived_acknowledge_ball_in_transit() {}

}  // namespace strategy
