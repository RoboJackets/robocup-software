#include "rj_strategy/agent/position/free_kicker.hpp"

namespace strategy {

FreeKicker::FreeKicker(int r_id) : Position(r_id, "FreeKicker") {}

FreeKicker::FreeKicker(Position&& other) : Position{std::move(other)} {
    position_name_ = "FreeKicker";
}

std::optional<RobotIntent> FreeKicker::derived_get_task(RobotIntent intent) {
    // Penalty Kicker kicks the ball into the goal

    // SPDLOG_INFO("Free Kicker {} is running", this->robot_id_);

    // if (last_world_state_->ball.position.dist_to(
    //         last_world_state_->get_robot(true, robot_id_).pose.position()) >= kOwnBallRadius) {
    //     rj_geometry::Point ball_position = last_world_state_->ball.position;
    //     rj_geometry::Point their_goal = field_dimensions_.their_goal_loc();
    //     rj_geometry::Point goal_to_ball = (ball_position - their_goal).normalized();
    //     rj_geometry::Point steal_point =
    //         ball_position + goal_to_ball * kStealApproachDistance;

    //     auto collect_cmd = planning::MotionCommand{
    //         "path_target", planning::LinearMotionInstant{steal_point}, planning::FaceBall{}};
    //     intent.motion_command = collect_cmd;

    // }

    planning::LinearMotionInstant target{calculate_best_shot(last_world_state_, field_dimensions_)};
    auto line_kick_cmd = planning::MotionCommand{"line_kick", target};
    intent.motion_command = line_kick_cmd;

    // note: the way this is set up makes it impossible to
    // shoot on time without breakbeam
    intent.shoot_mode = RobotIntent::ShootMode::KICK;
    intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
    intent.kick_speed = max_kick_speed();
    intent.is_active = true;

    return intent;
}

std::string FreeKicker::get_current_state() { return "FreeKicker"; }

void FreeKicker::derived_acknowledge_pass() {}

void FreeKicker::derived_pass_ball() {}

void FreeKicker::derived_acknowledge_ball_in_transit() {}

}  // namespace strategy
