#include "line.hpp"
#include "planning/instant.hpp"
#include "planning/planner/motion_command.hpp"

namespace strategy {

Line::Line(const Position& other) : Position{other} { position_name_ = "Line"; }

Line::Line(int r_id) : Position{r_id, "Line"} {}

Line::Line(int r_id, bool vertical) : Position{r_id, "Line"}, vertical_{vertical} {}

std::optional<RobotIntent> Line::derived_get_task(RobotIntent intent) {
    if (check_is_done()) {
        forward_ = !forward_;
    }

    if (vertical_) {
        if (forward_) {
            auto motion_command = planning::MotionCommand{
                "path_target",
                planning::LinearMotionInstant{
                    rj_geometry::Point{
                        field_dimensions_.center_field_loc().x() - (robot_id_ - 3) * 1,
                        field_dimensions_.center_field_loc().y() - 2.5 + 5 * 0.75,
                    },
                    rj_geometry::Point{0.0, 0.0},
                },
                planning::FaceTarget(), true};

            intent.motion_command = motion_command;
        } else {
            auto motion_command = planning::MotionCommand{
                "path_target",
                planning::LinearMotionInstant{
                    rj_geometry::Point{
                        field_dimensions_.center_field_loc().x() - (robot_id_ - 3) * 1,
                        field_dimensions_.center_field_loc().y() - 4.5 + 5 * 0.75,
                    },
                    rj_geometry::Point{0.0, 0.0},
                },
                planning::FaceTarget(), true};

            intent.motion_command = motion_command;
        }
    } else {
        auto motion_command = planning::MotionCommand{
            "rotate",planning::LinearMotionInstant{{0.0, 0.0}, {0.0, 0.0}}};

        intent.motion_command = motion_command;
    }

    return intent;
}
}  // namespace strategy
