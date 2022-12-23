#include "offense.hpp"

namespace strategy {

Offense::Offense(int r_id) : Position(r_id) { position_name_ = "Offense"; }

std::optional<RobotIntent> Offense::derived_get_task(RobotIntent intent) {
    // FSM: kick -> move away -> repeat
    if (check_is_done()) {
        // switch from kicking -> not kicking or vice versa
        kicking_ = !kicking_;
    }

    // get world_state
    WorldState* world_state = this->world_state();  // thread-safe getter

    // oscillate along vertical line (temp)
    auto ptc = planning::PathTargetCommand{};
    double y = 6.0;
    if (!kicking_) {
        y = 1.0;
    }
    rj_geometry::Point back_pt{(6.0 * (robot_id_ * 0.1) - 2.0), y};
    ptc.goal.position = back_pt;

    rj_geometry::Point vel{0.0, 0.0};
    ptc.goal.velocity = vel;

    rj_geometry::Point ball_pos = world_state->ball.position;
    planning::TargetFacePoint face_pt = planning::TargetFacePoint{};
    face_pt.face_point = ball_pos;

    ptc.angle_override = face_pt;
    ptc.ignore_ball = false;

    intent.motion_command = ptc;
    return intent;
}

}  // namespace strategy
