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
    double y = 6.0;
    if (!kicking_) {
        y = 1.0;
    }
    rj_geometry::Point target_pt{(6.0 * (robot_id_ * 0.1) - 2.0), y};

    // stop at end of path
    rj_geometry::Point target_vel{0.0, 0.0};

    // face ball on way up, face path on way down
    planning::AngleOverride angle_override = planning::TargetFacePoint{world_state->ball.position};
    if (kicking_) {
        angle_override = planning::TargetFaceTangent{};
    }

    // avoid ball
    bool ignore_ball = false;

    planning::LinearMotionInstant goal{target_pt, target_vel};
    intent.motion_command = planning::PathTargetCommand{goal, angle_override, ignore_ball};
    intent.motion_command_name = "path_target";
    return intent;
}

}  // namespace strategy
