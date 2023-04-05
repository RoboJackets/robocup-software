#include "penalty_kicker.hpp"

namespace strategy {

PenaltyKicker::PenaltyKicker(int r_id) : Position(r_id) { position_name_ = "PenaltyKicker"; }

std::optional<RobotIntent> PenaltyKicker::derived_get_task(RobotIntent intent) {
    // Penalty Kicker lines up with the ball
    // Only robot allowed within 1 m of the ball
    // Cannot touch the ball
    // https://robocup-ssl.github.io/ssl-rules/sslrules.html#_penalty_kick

    WorldState* world_state = this->world_state();

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
    intent.motion_command = planning::MotionCommand{"path_target", goal, face_option, ignore_ball};
    return intent;
}

}  // namespace strategy