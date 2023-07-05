#include "marker.hpp"

namespace strategy {
Marker::Marker() {}
std::optional<RobotIntent> Marker::get_task(RobotIntent intent,
                                            const WorldState* world_state, FieldDimensions field_dimensions) {
    auto ball_position = world_state->ball.position;

    int closestId = 0;
    int closestDist = 9999999;
    //FInds closest enemy to goal
    for(int i = 0; i < kNumShells; i++) {
        RobotState robot = world_state->get_robot(false, i);
        if(robot.visible) {
            if((robot.pose.position()-ball_position).mag() < closestDist) {
                closestId = i;
                closestDist = (robot.pose.position()-ball_position).mag();
            }
        }
    }
    // face ball
    planning::PathTargetFaceOption face_option = planning::FaceBall{};
    // planning::LinearMotionInstant goal{
    //     (world_state->ball.position - ball_position) * factor + ball_position,
    //     rj_geometry::Point{0.0, 0.0}};
    auto target_position = (world_state->get_robot(false, closestId).pose.position() - ball_position) * factor + ball_position;
    planning::LinearMotionInstant goal{target_position, rj_geometry::Point{0.0, 0.0}};
    
    intent.motion_command = planning::MotionCommand{"path_target", goal, face_option, false};
    return intent;
}
}  // namespace strategy