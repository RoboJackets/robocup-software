#include "line.hpp"

namespace strategy {

Line::Line(const Position& other) : Position{other} { position_name_ = "SoloOffense"; }

Line::Line(int r_id) : Position{r_id, "SoloOffense"} {}

std::optional<RobotIntent> Line::derived_get_task(RobotIntent intent) {
    if (check_is_done()) {
        forward_ = !forward_;
    }

    if (forward_) {
        auto motion_command = planning::MotionCommand{
            "path_target",
            planning::LinearMotionInstant{
                rj_geometry::Point{
                    field_dimensions_.center_field_loc().x() - 1.0,
                    field_dimensions_.center_field_loc().y() - 2.0 + robot_id_ * 0.75},
                rj_geometry::Point{0.0, 0.0}},
            planning::FacePoint{rj_geometry::Point{
                field_dimensions_.center_field_loc().x() - 1.0,
                field_dimensions_.center_field_loc().y() - 2.0 + robot_id_ * 0.75}},
            true};

        intent.motion_command = motion_command;
    } else {
        auto motion_command = planning::MotionCommand{
            "path_target",
            planning::LinearMotionInstant{
                rj_geometry::Point{
                    field_dimensions_.center_field_loc().x() + 1.0,
                    field_dimensions_.center_field_loc().y() - 2.0 + robot_id_ * 0.75},
                rj_geometry::Point{0.0, 0.0}},
            planning::FacePoint{rj_geometry::Point{
                field_dimensions_.center_field_loc().x() - 1.0,
                field_dimensions_.center_field_loc().y() - 2.0 + robot_id_ * 0.75}},
            true};

        intent.motion_command = motion_command;
    }

    return intent;
}
}  // namespace strategy
