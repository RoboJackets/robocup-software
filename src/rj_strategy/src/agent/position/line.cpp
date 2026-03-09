#include "rj_strategy/agent/position/line.hpp"

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
    if (check_is_done()) {
        forward_ = !forward_;
    }

    if (forward_) {
        // move to start
        auto motion_command = planning::MotionCommand{"path_target",
                                                      planning::LinearMotionInstant{
                                                          start_,
                                                          rj_geometry::Point{0.0, 0.0},
                                                      },
                                                      planning::FaceTarget(), true};
        intent.motion_command = motion_command;
    } else {
        // move to end
        auto motion_command = planning::MotionCommand{"path_target",
                                                      planning::LinearMotionInstant{
                                                          end_,
                                                          rj_geometry::Point{0.0, 0.0},
                                                      },
                                                      planning::FaceTarget(), true};
        intent.motion_command = motion_command;
    }

    return intent;
}
}  // namespace strategy
