#include "rj_strategy/agent/position/line.hpp"

namespace strategy {

Line::Line(Position&& other) : Position{std::move(other)} { position_name_ = "Line"; }

Line::Line(int r_id) : Position{r_id, "Line"} {}

Line::Line(int r_id, bool vertical) : Position{r_id, "Line"}, vertical_{vertical} {}

std::optional<RobotIntent> Line::derived_get_task(RobotIntent intent) {
    if (check_is_done()) {
        forward_ = !forward_;
        last_time_ = RJ::now();
    }

    if (RJ::now() - last_time_ < RJ::Seconds{1.5}) {
        return intent;
    }

    if (vertical_) {
        if (forward_) {
            auto motion_command = planning::MotionCommand{
                "rotate",
                planning::LinearMotionInstant{
                    rj_geometry::Point{
                        field_dimensions_.center_field_loc().x(),
                        field_dimensions_.center_field_loc().y()
                    },
                    rj_geometry::Point{0.0, 0.0},
                },
                planning::FaceTarget(), true};

            intent.motion_command = motion_command;
        } else {
            auto motion_command = planning::MotionCommand{
                "rotate",
                planning::LinearMotionInstant{
                    field_dimensions_.their_goal_loc(),
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
                            (field_dimensions_.center_field_loc().y() - 1) / 6 * robot_id_ + 1 + 0.5,
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
                            (field_dimensions_.center_field_loc().y() - 1) / 6 * robot_id_ + 1 + 0.5,
                        },
                        rj_geometry::Point{0.0, 0.0},
                    },
                    planning::FaceAngle{0}, true};

                intent.motion_command = motion_command;
            }

        } else {
            if (face_target_) {
                auto motion_command = planning::MotionCommand{
                    "path_target",
                    planning::LinearMotionInstant{
                        rj_geometry::Point{
                            field_dimensions_.our_defense_area().minx(),
                            (field_dimensions_.center_field_loc().y() - 1) / 6 * robot_id_ + 1 + 0.50,
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
                            field_dimensions_.our_defense_area().minx(),
                            (field_dimensions_.center_field_loc().y() - 1) / 6 * robot_id_ + 1 + 0.50,
                        },
                        rj_geometry::Point{0.0, 0.0},
                    },
                    planning::FaceAngle{0}, true};

                intent.motion_command = motion_command;
            }
        }
    }

    return intent;
}
}  // namespace strategy
