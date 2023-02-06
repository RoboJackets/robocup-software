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
    double x = -3.0;
    if (move_ct_ % 2 == 1) {
        x = 3.0;
    }
    rj_geometry::Point target_pt{x, robot_id_ * 1.0};

    // stop at end of path
    rj_geometry::Point target_vel{0.0, 0.0};

    // face ball
    planning::PathTargetFaceOption face_option = planning::FaceBall{};

    // avoid ball
    bool ignore_ball = false;

    planning::LinearMotionInstant goal{target_pt, target_vel};
    intent.motion_command = planning::PathTargetMotionCommand{goal, face_option, ignore_ball};
    intent.motion_command_name = "path_target";
    return intent;
}

}  // namespace strategy
