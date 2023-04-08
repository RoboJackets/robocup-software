#include "penalty_kicker.hpp"

namespace strategy {

std::optional<RobotIntent> PenaltyKicker::get_task(RobotIntent intent,
                                                   const WorldState* world_state,
                                                   FieldDimensions field_dimensions) {
    // Penalty Kicker kicks the ball into the goal

    rj_geometry::Point goal_corner{
        field_dimensions.their_goal_loc().x() + 0.5 * field_dimensions.goal_width(),
        field_dimensions.their_goal_loc().y()};

    planning::LinearMotionInstant target{goal_corner};
    auto line_kick_cmd = planning::MotionCommand{"line_kick", target};
    intent.motion_command = line_kick_cmd;

    intent.shoot_mode = RobotIntent::ShootMode::CHIP;
    intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
    intent.kick_speed = 4.0;
    intent.is_active = true;

    return intent;
}

}  // namespace strategy