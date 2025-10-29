#include "rj_strategy/agent/position/free_kicker.hpp"

namespace strategy {

FreeKicker::FreeKicker(int r_id) : Position(r_id, "FreeKicker") {}

FreeKicker::FreeKicker(const Position& other) : Position{other} { position_name_ = "FreeKicker"; }

std::optional<RobotIntent> FreeKicker::derived_get_task(RobotIntent intent) {
    // Penalty Kicker kicks the ball into the goal

    // SPDLOG_INFO("Free Kicker {} is running", this->robot_id_);

    // Read positions of their team
    std::vector<RobotState> their_robots = this->last_world_state_->their_robots;
    rj_geometry::Point enemy_goalie_location = this->field_dimensions_.their_goal_loc();

    for (const RobotState& enemy : their_robots) {
        // Get position of their goalie
        if (this->field_dimensions_.their_defense_area().hit(enemy.pose.position())) {
            enemy_goalie_location = enemy.pose.position();
            break;
        }
    }

    rj_geometry::Point best_shot = this->field_dimensions_.their_goal_loc();
    rj_geometry::Point increment(0.05, 0);
    double best_distance = -1.0;
    double goal_width = field_dimensions_.goal_width();
    rj_geometry::Point curr_point =
        field_dimensions_.their_goal_loc() - rj_geometry::Point(goal_width / 2.0, 0) + increment;
    rj_geometry::Point ball_position = this->last_world_state_->ball.position;
    rj_geometry::Point vec = curr_point - ball_position;
    for (int i = 0; i < 19; i++) {
        rj_geometry::Point enemy_vec = enemy_goalie_location - curr_point;
        auto projection = (enemy_vec.dot(vec) / vec.dot(vec));
        enemy_vec = enemy_vec - (projection)*vec;
        double distance = enemy_vec.mag();

        if (distance > best_distance) {
            best_distance = distance;
            best_shot = curr_point;
        }
        curr_point = curr_point + increment;
    }

    planning::LinearMotionInstant target{best_shot};
    auto line_kick_cmd = planning::MotionCommand{"line_kick", target};
    intent.motion_command = line_kick_cmd;

    // note: the way this is set up makes it impossible to
    // shoot on time without breakbeam
    intent.shoot_mode = RobotIntent::ShootMode::CHIP;
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
