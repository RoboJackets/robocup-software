#include "penalty_player.hpp"

namespace strategy {

PenaltyPlayer::PenaltyPlayer(int r_id) : Position(r_id) { position_name_ = "PenaltyPlayer"; }

std::optional<RobotIntent> PenaltyPlayer::derived_get_task(RobotIntent intent) {
    // Penalty Player lines up with the ball
    // Only robot allowed within 1 m of the ball
    // Cannot touch the ball
    // https://robocup-ssl.github.io/ssl-rules/sslrules.html#_penalty_kick
    // Line up 0.15 meters behind the ball

    WorldState* world_state = last_world_state_;

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

std::string PenaltyPlayer::return_current_state() {
    return "PenaltyPlayer";
}

void PenaltyPlayer::derived_acknowledge_pass() {}

void PenaltyPlayer::derived_pass_ball() {}

void PenaltyPlayer::derived_acknowledge_ball_in_transit() {}

}  // namespace strategy
