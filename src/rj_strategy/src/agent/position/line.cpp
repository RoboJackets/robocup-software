#include "rj_strategy/agent/position/line.hpp"
#include <rj_common/planning/motion_command.hpp>
#include <rj_common/world_state.hpp>

namespace strategy {

Line::Line(Position&& other) : Position{std::move(other)} { position_name_ = "Line"; }

Line::Line(int r_id) : Position{r_id, "Line"} {}

Line::Line(int r_id, bool vertical) : Position{r_id, "Line"}, vertical_{vertical} {}

Line::Line(int r_id, rj_geometry::Point start, rj_geometry::Point end, uint8_t target_rid)
    : Position{r_id, "Line"}, start_{start}, end_{end}, target_rid_{target_rid} {}

std::optional<RobotIntent> Line::derived_get_task(RobotIntent intent) {
    if (robot_id_ != target_rid_) {
        return intent;
    }

    // toggles direction if motion complete
    // if (check_is_done()) {
    //     forward_ += 1;
    // }

    if (forward_ == 0) {
        // move to start
        auto motion_command = planning::MotionCommand{"rotate",
                                                      planning::LinearMotionInstant{
                                                          start_,
                                                          rj_geometry::Point{0.0, 0.0},
                                                      }};
        intent.motion_command = motion_command;
    }
    // } else if (forward_ == 1) {
    //     // move to end
    //     auto motion_command = planning::MotionCommand{"path_target",
    //                                                   planning::LinearMotionInstant{
    //                                                       end_,
    //                                                       rj_geometry::Point{0.0, 0.0},
    //                                                   },
    //                                                   planning::FacePoint{rj_geometry::Point{-1.0, 7.0}}, true};
    //     intent.motion_command = motion_command;
    // } else if (forward_ == 2) {
    //     // move to end
    //     auto motion_command = planning::MotionCommand{"rotate",
    //                                                   planning::LinearMotionInstant{
    //                                                       rj_geometry::Point{-1.0, 8.0},
    //                                                       rj_geometry::Point{0.0, 0.0},
    //                                                   },
    //                                                   planning::FacePoint{rj_geometry::Point{-1.0, 8.0}}, true};
    //     intent.motion_command = motion_command;
    // }

    return intent;
}
}  // namespace strategy
