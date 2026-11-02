#include "rj_strategy/agent/position/runner.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position{r_id, "Runner"} {}

Runner::Runner(Position&& other) : Position{std::move(other)} { position_name_ = "Runner"; }

std::string Runner::get_current_state() {
    return std::string{"Runner "} + std::string{state_to_name(current_state_)};
}

Runner::State Runner::next_state() {
    switch (current_state_) {
        case CORNER_1:
            return CORNER_2;
        case CORNER_2:
            return CORNER_3;
        case CORNER_3:
            return CORNER_4;
        case CORNER_4:
            return CORNER_1;
    }
}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    rj_geometry::Point center = field_dimensions_.center_field_loc();

    double loop_half_width = field_dimensions_.width() * loopPercentage / 2;
    double loop_half_length = field_dimensions_.length() * loopPercentage / 2;

    // Coords of the corners
    std::array<rj_geometry::Point, 4> loopCorners{
        center + rj_geometry::Point{-loop_half_width, -loop_half_length},
        center + rj_geometry::Point{-loop_half_width, loop_half_length},
        center + rj_geometry::Point{loop_half_width, loop_half_length},
        center + rj_geometry::Point{loop_half_width, -loop_half_length}};

    rj_geometry::Point robot_position =
        last_world_state_->get_robot(true, robot_id_).pose.position();

    // Once arrived at a corner go to next
    if (robot_position.dist_to(loopCorners[current_state_]) < cornerDistTolerance) {
        current_state_ = next_state();
    }

    planning::LinearMotionInstant target{loopCorners[current_state_], rj_geometry::Point{0.0, 0.0}};
    intent.motion_command =
        planning::MotionCommand{"path_target", target, planning::FaceTarget{}};
    return intent;
}

}  // namespace strategy