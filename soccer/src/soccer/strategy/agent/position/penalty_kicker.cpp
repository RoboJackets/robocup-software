#include "penalty_kicker.hpp"

namespace strategy {

std::optional<RobotIntent> PenaltyKicker::get_task(RobotIntent intent,
                                                   const WorldState* world_state,
                                                   FieldDimensions field_dimensions) {
    // Penalty Kicker lines up with the ball
    // Only robot allowed within 1 m of the ball
    // Cannot touch the ball
    // https://robocup-ssl.github.io/ssl-rules/sslrules.html#_penalty_kick

    // Line up 0.15 meters behind the ball
    rj_geometry::Point target_pt{world_state->ball.position.x(),
                 world_state->ball.position.y() - kRobotRadius - 0.15};

    // Stop at end of path
    rj_geometry::Point target_vel{0.0, 0.0};

    // Face ball
    planning::PathTargetFaceOption face_option{planning::FaceBall{}};

    // Avoid ball
    bool ignore_ball{false};

    // Create Motion Command
    planning::LinearMotionInstant goal{target_pt, target_vel};
    intent.motion_command = planning::PathTargetMotionCommand{goal, face_option, ignore_ball};
    intent.motion_command_name = "path_target";
    return intent;
}

}  // namespace strategy