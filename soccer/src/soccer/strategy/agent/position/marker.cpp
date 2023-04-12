#include "marker.hpp"

namespace strategy {
Marker::Marker(double factor) : factor{factor} {}
std::optional<RobotIntent> Marker::get_task(RobotIntent intent, const WorldState* const world_state,
                                            FieldDimensions field_dimensions) {
    auto goal_center = field_dimensions.our_goal_loc();  // TODO: replace with field coordinates

    int closestId = 0;
    int closestDist = 9999999;
    // FInds closest enemy to goal
    for (int i = 0; i < kNumShells; i++) {
        RobotState robot = world_state->get_robot(false, i);
        if (robot.visible) {
            if ((robot.pose.position() - goal_center).mag() < closestDist) {
                closestId = i;
                closestDist = (robot.pose.position() - goal_center).mag();
            }
        }
    }
    // face ball
    planning::PathTargetFaceOption face_option = planning::FaceBall{};
    // avoid ball
    constexpr bool IGNORE_BALL = false;
    // planning::LinearMotionInstant goal{
    //     (world_state->ball.position - goal_center) * factor + goal_center,
    //     rj_geometry::Point{0.0, 0.0}};
    planning::LinearMotionInstant goal{
        (world_state->get_robot(false, closestId).pose.position() - goal_center) * factor +
            goal_center,
        rj_geometry::Point{0.0, 0.0}};

    intent.motion_command = planning::MotionCommand{"path_target", goal, face_option, IGNORE_BALL};
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
    intent.motion_command = planning::MotionCommand{"path_target", goal, face_option, IGNORE_BALL};
    return intent;
}
}  // namespace strategy