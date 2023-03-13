#include "marker.hpp"

namespace strategy {
Marker::Marker(double factor) : factor{factor} {}
std::optional<RobotIntent> Marker::get_task(RobotIntent intent,
                                            const WorldState* const world_state) {
    // face ball
    planning::PathTargetFaceOption face_option = planning::FaceBall{};
    // avoid ball
    constexpr bool IGNORE_BALL = false;
    auto goal_center = rj_geometry::Point{0.0, 0.0};  // TODO: replace with field coordinates
    planning::LinearMotionInstant goal{
        (world_state->ball.position - goal_center) * factor + goal_center,
        rj_geometry::Point{0.0, 0.0}};
    intent.motion_command = planning::PathTargetMotionCommand{goal, face_option, IGNORE_BALL};
    intent.motion_command_name = "marker_target";
    return intent;
}
std::optional<RobotIntent> Marker::get_point_task(RobotIntent intent,
                                                  const rj_geometry::Point& point1,
                                                  const rj_geometry::Point& point2) {
    // face ball
    planning::PathTargetFaceOption face_option = planning::FaceBall{};
    // avoid ball
    constexpr bool IGNORE_BALL = false;
    planning::LinearMotionInstant goal{(point2 - point1) * factor + point1,
                                       rj_geometry::Point{0.0, 0.0}};
    intent.motion_command = planning::PathTargetMotionCommand{goal, face_option, IGNORE_BALL};
    intent.motion_command_name = "marker_target";
    return intent;
}
}  // namespace strategy