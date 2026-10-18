#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position{r_id, "Runner"} {}

Runner::Runner(Position&& other) : Position{std::move(other)} { position_name_ = "Runner"; }

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    // Same tick order as Offense: pick the state, then build the intent for it.
    current_state_ = next_state();
    return state_to_task(intent);
}

std::string Runner::get_current_state() { return "Runner " + get_state_name(); }

std::string Runner::get_state_name() const {
    switch (current_state_) {
        case SIDE_ONE:
            return "SIDE_ONE";
        case SIDE_TWO:
            return "SIDE_TWO";
        case SIDE_THREE:
            return "SIDE_THREE";
        case SIDE_FOUR:
            return "SIDE_FOUR";
    }
    return "SIDE_ONE";
}

Runner::State Runner::next_state() {
    // check_is_done() is set by the agent when the planner finishes this path.
    // Reading it clears the flag, so only call it here.
    if (!check_is_done()) {
        return current_state_;
    }

    // Finished this side, so start the next one. After side four, begin again.
    switch (current_state_) {
        case SIDE_ONE:
            return SIDE_TWO;
        case SIDE_TWO:
            return SIDE_THREE;
        case SIDE_THREE:
            return SIDE_FOUR;
        case SIDE_FOUR:
            return SIDE_ONE;
    }
    return SIDE_ONE;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    // Field size arrives from the config server. Until then, stay put.
    if (field_dimensions_.length() <= 0.0f) {
        intent.motion_command = planning::MotionCommand{"halt"};
        return intent;
    }

    // "path_target" is the straight-line planner. Stop at the corner (velocity 0)
    // so the planner can report done. ignore_ball lets the lap continue if the
    // ball is sitting on the rectangle. FaceTarget turns the robot along the path.
    intent.motion_command = planning::MotionCommand{
        "path_target", planning::LinearMotionInstant{corner_for(current_state_)},
        planning::FaceTarget{}, true};
    return intent;
}

rj_geometry::Point Runner::corner_for(State state) const {
    // Half a meter inside the sidelines, and half a meter past each defense area.
    const double margin = 0.5;
    const double y_pad = field_dimensions_.penalty_short_dist() + margin;
    const double x_left = field_dimensions_.field_x_left_coord() + margin;
    const double x_right = field_dimensions_.field_x_right_coord() - margin;
    const double y_near = y_pad;
    const double y_far = field_dimensions_.length() - y_pad;

    // Order is a counterclockwise lap: our-left, our-right, their-right, their-left.
    switch (state) {
        case SIDE_ONE:
            return rj_geometry::Point{x_left, y_near};
        case SIDE_TWO:
            return rj_geometry::Point{x_right, y_near};
        case SIDE_THREE:
            return rj_geometry::Point{x_right, y_far};
        case SIDE_FOUR:
            return rj_geometry::Point{x_left, y_far};
    }
    return rj_geometry::Point{x_left, y_near};
}

}  // namespace strategy
