#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position{r_id, "Runner"} {}

Runner::Runner(Position&& other) : Position{std::move(other)} { position_name_ = "Runner"; }

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    current_state_ = next_state();
    return state_to_task(intent);
}

std::string Runner::get_current_state() { return "Runner " + get_state_name(); }

std::string Runner::get_state_name() const {
    switch (current_state_) {
        case DEFAULT:
            return "DEFAULT";
        case SIDE_ONE:
            return "SIDE_ONE";
        case SIDE_TWO:
            return "SIDE_TWO";
        case SIDE_THREE:
            return "SIDE_THREE";
        case SIDE_FOUR:
            return "SIDE_FOUR";
    }
    return "DEFAULT";
}

Runner::State Runner::next_state() {
    if (field_dimensions_.length() <= 0.0f) {
        return DEFAULT;
    }

    if (current_state_ == DEFAULT) {
        return SIDE_ONE;
    }

    const bool planner_done = check_is_done();
    if (!planner_done && !reached_current_target()) {
        return current_state_;
    }

    switch (current_state_) {
        case SIDE_ONE:
            return SIDE_TWO;
        case SIDE_TWO:
            return SIDE_THREE;
        case SIDE_THREE:
            return SIDE_FOUR;
        case SIDE_FOUR:
            return SIDE_ONE;
        case DEFAULT:
            return SIDE_ONE;
    }
    return SIDE_ONE;
}

bool Runner::reached_current_target() const {
    if (!last_world_state_ || current_state_ == DEFAULT) {
        return false;
    }
    return last_world_state_->get_robot(true, robot_id_)
               .pose.position()
               .dist_to(corner_for(current_state_)) < 0.3;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    if (current_state_ == DEFAULT || field_dimensions_.length() <= 0.0f) {
        intent.motion_command = planning::MotionCommand{"halt"};
        return intent;
    }

    intent.motion_command = planning::MotionCommand{
        "path_target", planning::LinearMotionInstant{corner_for(current_state_)},
        planning::FaceTarget{}, true};
    return intent;
}

rj_geometry::Point Runner::corner_for(State state) const {
    const double margin = 0.5;
    const double y_pad = field_dimensions_.penalty_short_dist() + margin;
    const double x_left = field_dimensions_.field_x_left_coord() + margin;
    const double x_right = field_dimensions_.field_x_right_coord() - margin;
    const double y_near = y_pad;
    const double y_far = field_dimensions_.length() - y_pad;

    switch (state) {
        case SIDE_ONE:
            return rj_geometry::Point{x_left, y_near};
        case SIDE_TWO:
            return rj_geometry::Point{x_right, y_near};
        case SIDE_THREE:
            return rj_geometry::Point{x_right, y_far};
        case SIDE_FOUR:
            return rj_geometry::Point{x_left, y_far};
        case DEFAULT:
            break;
    }
    return rj_geometry::Point{x_left, y_near};
}

}  