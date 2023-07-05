#include "robot_marker.hpp"

namespace strategy {
RobotMarker::RobotMarker(u_int8_t robot_id) {
    this->target_robot_id = robot_id;
}

std::optional<RobotIntent> RobotMarker::get_task(RobotIntent intent,
                                            const WorldState* world_state, FieldDimensions field_dimensions) {
    auto ball_position = world_state->ball.position;

    // face ball
    planning::PathTargetFaceOption face_option = planning::FaceBall{};
    // planning::LinearMotionInstant goal{
    //     (world_state->ball.position - ball_position) * factor + ball_position,
    //     rj_geometry::Point{0.0, 0.0}};
    auto target_position = (world_state->get_robot(false, target_robot_id).pose.position() - ball_position) * factor + ball_position;
    planning::LinearMotionInstant goal{target_position, rj_geometry::Point{0.0, 0.0}};
    
    intent.motion_command = planning::MotionCommand{"path_target", goal, face_option, false};
    return intent;
}
}  // namespace strategy