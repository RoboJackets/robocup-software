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

    constexpr double BALL_WIDTH_OFFSET = kBallRadius * 10;
    rj_geometry::Point const right_goal_post =
        this->field_dimensions_.their_goal_loc() +
        rj_geometry::Point((this->field_dimensions_.goal_width() / 2.0) - BALL_WIDTH_OFFSET, 0);

    rj_geometry::Point const left_goal_post =
        this->field_dimensions_.their_goal_loc() -
        rj_geometry::Point((this->field_dimensions_.goal_width() / 2.0) - BALL_WIDTH_OFFSET, 0);

    double const left_dist = get_shot_dist_to_goalie(enemy_goalie_location, left_goal_post);
    double const right_dist = get_shot_dist_to_goalie(enemy_goalie_location, right_goal_post);
    rj_geometry::Point const best_shot = left_dist < right_dist ? right_goal_post : left_goal_post;

    /*
    LOGIC FOR PASSING WHEN SHOT TOO EXTREME (i.e. CORNER KICK)

    double const shot_angle = abs((best_shot - this->last_world_state_->ball.position).angle());

    if (shot_angle > 3 * M_PI / 4.0 ||
        shot_angle < M_PI / 4.0) {
        SPDLOG_INFO("Free Kicker {}: Shot angle {} too extreme, not shooting", this->robot_id_,
           shot_angle);
        // PASS
        return intent;
    }

    SPDLOG_INFO("Free Kicker {} shooting at point {}, {} with angle {}", this->robot_id_,
    best_shot.x(), best_shot.y(), shot_angle);
    */

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

double FreeKicker::get_shot_dist_to_goalie(rj_geometry::Point const& goalie_pos,
                                           rj_geometry::Point const& shot_target) {
    rj_geometry::Point const ball_position = this->last_world_state_->ball.position;
    return std::abs((goalie_pos - ball_position).cross(shot_target - ball_position)) /
           (shot_target - ball_position).mag();
}
}  // namespace strategy