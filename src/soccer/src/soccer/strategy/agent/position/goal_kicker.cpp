#include "goal_kicker.hpp"

namespace strategy {

GoalKicker::GoalKicker(int r_id) : Position(r_id, "GoalKicker") {}

std::optional<RobotIntent> GoalKicker::derived_get_task(RobotIntent intent) {
    // Penalty Kicker kicks the ball into the goal

    SPDLOG_INFO("Goal Kicker {} is running", this->robot_id_);

    rj_geometry::Point goal_corner{
        this->field_dimensions_.their_goal_loc().x() + 0.5 * this->field_dimensions_.goal_width(),
        this->field_dimensions_.their_goal_loc().y()};

    planning::LinearMotionInstant target{goal_corner};
    auto line_kick_cmd = planning::MotionCommand{"line_kick", target};
    intent.motion_command = line_kick_cmd;

    // note: the way this is set up makes it impossible to
    // shoot on time without breakbeam
    // TODO(Kevin): make intent hold a manip msg instead? to be cleaner?
    intent.shoot_mode = RobotIntent::ShootMode::CHIP;
    intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
    intent.kick_speed = 4.0;
    intent.is_active = true;

    return intent;
}

std::string GoalKicker::get_current_state() { return "GoalKicker"; }

void GoalKicker::derived_acknowledge_pass() {}

void GoalKicker::derived_pass_ball() {}

void GoalKicker::derived_acknowledge_ball_in_transit() {}

}  // namespace strategy