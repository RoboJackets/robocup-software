#include "defense.hpp"

namespace strategy {

Defense::Defense(int r_id) : Position(r_id) { position_name_ = "Defense"; }

std::optional<RobotIntent> Defense::derived_get_task(RobotIntent intent) {
    if (check_is_done()) {
        // toggle move pts
        move_ct_++;
    }

    // thread-safe getter
    WorldState* world_state = this->world_state();

    // oscillate along horizontal line (temp)
    auto ptc = planning::PathTargetCommand{};
    double x = -3.0;
    if (move_ct_ % 2 == 1) {
        x = 3.0;
    }

    rj_geometry::Point pt{x, 3.0};
    ptc.goal.position = pt;

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
