#include "line.hpp"

namespace strategy {

Line::Line(const Position& other) : Position{other} { position_name_ = "Line"; }

Line::Line(int r_id) : Position{r_id, "Line"} { this->robot_id_ = r_id; }

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
        if (forward_) {
            if (face_target_) {
                auto motion_command = planning::MotionCommand{
                    "path_target",
                    planning::LinearMotionInstant{
                        rj_geometry::Point{
                            field_dimensions_.center_field_loc().x() - 2.5 + 5 * 0.75,
                            (field_dimensions_.center_field_loc().y() - 1) / 6 * robot_id_ + 1,
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
                            field_dimensions_.our_defense_area().maxx(),
                            (field_dimensions_.center_field_loc().y() - 1) / 6 * robot_id_ + 1,
                        },
                        rj_geometry::Point{0.0, 0.0},
                    },
                    planning::FacePoint{rj_geometry::Point{
                        field_dimensions_.field_x_left_coord(),
                        (field_dimensions_.center_field_loc().y() - 1) / 6 * robot_id_ + 1,
                    }},
                    true};

                intent.motion_command = motion_command;
            }

        } else {
            if (face_target_) {
                auto motion_command = planning::MotionCommand{
                    "path_target",
                    planning::LinearMotionInstant{
                        rj_geometry::Point{
                            field_dimensions_.our_defense_area().minx(),
                            (field_dimensions_.center_field_loc().y() - 1) / 6 * robot_id_ + 1,
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
                            field_dimensions_.center_field_loc().x() - 4.5 + 5 * 0.75,
                            (field_dimensions_.center_field_loc().y() - 1) / 6 * robot_id_ + 1,
                        },
                        rj_geometry::Point{0.0, 0.0},
                    },
                    planning::FacePoint{rj_geometry::Point{
                        field_dimensions_.field_x_left_coord(),
                        (field_dimensions_.center_field_loc().y() - 1) / 6 * robot_id_ + 1,
                    }},
                    true};

                intent.motion_command = motion_command;
            }
        }
    }

    return intent;
}
}  // namespace strategy
