#include "runner.hpp"

#include <optional>

namespace strategy {

Runner::Runner(int r_id) : Position(r_id) { position_name_ = "Runner"; }

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) override {
    currentState_ = update_state();
    return state_to_task(intent);
}

Runner::State Runner::update_state() {
    auto next_state = currentState_;
    switch (currentState_) {
        case BOTTOM:
            if (check_is_done()) {
                next_state = RIGHT;
            }
            break;
        case RIGHT:
            if (check_is_done()) {
                next_state = TOP;
            }
            break;
        case TOP:
            if (check_is_done()) {
                next_state = LEFT;
            }
            break;
        case LEFT:
            if (check_is_done()) {
                next_state = BOTTOM;
            }
            break;
    }
    return next_state;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    rj_geometry::Point destination;

    if (currentState_ == BOTTOM) {
        destination = bottomRight_;
    } else if (currentState_ == RIGHT) {
        destination = topRight_;
    } else if (currentState_ == TOP) {
        destination = topLeft_;
    } else if (currentState_ == LEFT) {
        destination = bottomLeft_;
    }

    auto motion = planning::LinearMotionInstant{destination, rj_geometry::Point{0.0, 0.0}};
    auto motion_cmd = planning::MotionCommand{"path_target", motion, planning::FaceTarget{}, true};
    intent.motion_command = motion_cmd;
    return intent;
}

}  // namespace strategy