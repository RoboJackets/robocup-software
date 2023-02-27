#include "blocker.hpp"

namespace strategy {
Blocker::Blocker(double factor) : factor{factor} {}
std::optional<RobotIntent> Blocker::get_task(RobotIntent intent,
                                             const WorldState* const world_state) {
    // face ball
    planning::PathTargetFaceOption face_option = planning::FaceBall{};
    // avoid ball
    constexpr bool ignore_ball = false;
    auto goal_center = rj_geometry::Point{0.0, 0.0};  // TODO: replace with field coordinates
    planning::LinearMotionInstant goal{
        (world_state->ball.position - goal_center) * factor + goal_center,
        rj_geometry::Point{0.0, 0.0}};
    intent.motion_command = planning::PathTargetMotionCommand{goal, face_option, ignore_ball};
    intent.motion_command_name = "blocker_target";
    return intent;
}
}  // namespace strategy