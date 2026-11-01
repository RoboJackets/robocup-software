#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position{r_id, "Runner"} {}

Runner::Runner(Position&& other) : Position{std::move(other)} { position_name_ = "Runner"; }

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    // Decide which side we should be on, then build the command for it
    current_state_ = next_state();
    return state_to_task(intent);
}

std::string Runner::get_current_state() {
    return std::string{"Runner"} + std::string{state_to_name(current_state_)};
}

Runner::State Runner::next_state() {
    // Which corner are we heading to?
    rj_geometry::Point target;
    switch (current_state_) {
        case RUNNING_SIDE_1:
            target = corner_2_;
            break;
        case RUNNING_SIDE_2:
            target = corner_3_;
            break;
        case RUNNING_SIDE_3:
            target = corner_4_;
            break;
        case RUNNING_SIDE_4:
            target = corner_1_;
            break;
    }

    rj_geometry::Point robot_pos = last_world_state_->get_robot(true, robot_id_).pose.position();

    // check_is_done() clears the flag, so call it exactly once per tick.
    // Also treat "close enough to the corner" as reached, since is_done
    // may not fire when the same command is re-sent every tick.
    bool reached_corner = check_is_done() || robot_pos.dist_to(target) < 0.15;

    if (!reached_corner) {
        return current_state_;
    }

    // Reached the end of this side: move on to the next one
    switch (current_state_) {
        case RUNNING_SIDE_1:
            return RUNNING_SIDE_2;
        case RUNNING_SIDE_2:
            return RUNNING_SIDE_3;
        case RUNNING_SIDE_3:
            return RUNNING_SIDE_4;
        case RUNNING_SIDE_4:
            return RUNNING_SIDE_1;
    }
    return current_state_;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    // Each side ends at the next corner
    rj_geometry::Point target;
    switch (current_state_) {
        case RUNNING_SIDE_1:
            target = corner_2_;
            break;
        case RUNNING_SIDE_2:
            target = corner_3_;
            break;
        case RUNNING_SIDE_3:
            target = corner_4_;
            break;
        case RUNNING_SIDE_4:
            target = corner_1_;
            break;
    }

    // Drive in a straight line to the target corner
    auto run_cmd = planning::MotionCommand{"path_target", planning::LinearMotionInstant{target},
                                           planning::FaceBall{}};
    intent.motion_command = run_cmd;

    return intent;
}

}  // namespace strategy