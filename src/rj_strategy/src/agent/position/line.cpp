#include "rj_strategy/agent/position/line.hpp"

namespace strategy {

Line::Line(Position&& other) : Position{std::move(other)} { position_name_ = "Line"; }

Line::Line(int r_id) : Position{r_id, "Line"} {}

Line::Line(int r_id, bool vertical) : Position{r_id, "Line"}, vertical_{vertical} {}

Line::Line(int r_id, rj_geometry::Point start, rj_geometry::Point end)
    : Position {r_id, "Line"}
    , start_ {start}
    , end_ {end}
{}

std::optional<RobotIntent> Line::derived_get_task(RobotIntent intent) {
    if (check_is_done()) {
        forward_ = !forward_;
    }

    SPDLOG_INFO("TESTING START IN LINE.CPP: ({}, {})", start_[0], start_[1]);
    SPDLOG_INFO("TESTING END IN LINE.CPP: ({}, {})", end_[0], end_[1]);

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
                            field_dimensions_.our_defense_area().minx(),
                            (field_dimensions_.center_field_loc().y() - 1) / 6 * robot_id_ + 1,
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
