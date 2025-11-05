#include "rj_strategy/agent/position/free_kicker.hpp"

namespace strategy {

FreeKicker::FreeKicker(int r_id) : Position(r_id, "FreeKicker") {}

FreeKicker::FreeKicker(const Position& other) : Position{other} { position_name_ = "FreeKicker"; }

std::optional<RobotIntent> FreeKicker::derived_get_task(RobotIntent intent) {
    // Penalty Kicker kicks the ball into the goal

    // SPDLOG_INFO("Free Kicker {} is running", this->robot_id_);

    // Read positions of their team
    std::vector<RobotState> const their_robots = this->last_world_state_->their_robots;
    rj_geometry::Point enemy_goalie_location = this->field_dimensions_.their_goal_loc();

    for (const RobotState& enemy : their_robots) {
        // Get position of their goalie
        if (this->field_dimensions_.their_defense_area().hit(enemy.pose.position())) {
            enemy_goalie_location = enemy.pose.position();
            break;
        }
    }

    double ball_width_offset = 0.025;
    rj_geometry::Point const right_goal_post =
        this->field_dimensions_.their_goal_loc() + rj_geometry::Point((this->field_dimensions_.goal_width() / 2.0) - ball_width_offset, 0.0);
    
    rj_geometry::Point const left_goal_post =
        this->field_dimensions_.their_goal_loc() - rj_geometry::Point((this->field_dimensions_.goal_width() / 2.0) + ball_width_offset, 0.0);

    rj_geometry::Point best_shot = right_goal_post;
    double best_distance = -1.0;
    rj_geometry::Point ball_position = this->last_world_state_->ball.position;

    int num_samples = 20;

    for (int i = 0; i < num_samples; ++i) {
        double t = i / static_cast<double>(num_samples - 1);
        rj_geometry::Point shot_target = left_goal_post + (right_goal_post - left_goal_post) * t;

        double distance = std::abs((enemy_goalie_location - ball_position).cross(shot_target - ball_position)) /
                          (shot_target - ball_position).mag();

        if (distance > best_distance) {
            best_distance = distance;
            best_shot = shot_target;
        }
    }

    SPDLOG_INFO("Free Kicker {} shooting at point {}, {}", this->robot_id_, best_shot.x(), best_shot.y());

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
