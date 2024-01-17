#include "marker.hpp"

namespace strategy {
Marker::Marker(u_int8_t robot_id) {
    this->target_robot_id = robot_id;
}

std::optional<RobotIntent> Marker::get_task(RobotIntent intent,
                                            const WorldState* world_state, FieldDimensions field_dimensions) {
    auto ball_position = world_state->ball.position;

    u_int8_t cntr = 0;
    u_int8_t target_id = 0;
    for (u_int8_t other_id = 0; other_id < kNumShells; other_id++) {
        if (world_state->get_robot(false, other_id).visible) {
            if (cntr == target_robot_id) {
                target_id = other_id;
                break;
            }
            cntr++;
        }
    }

    // face ball
    planning::PathTargetFaceOption face_option = planning::FaceBall{};
    // planning::LinearMotionInstant goal{
    //     (world_state->ball.position - ball_position) * factor + ball_position,
    //     rj_geometry::Point{0.0, 0.0}};
    auto target_position = (world_state->get_robot(false, target_id).pose.position() - ball_position) * factor + ball_position;
    planning::LinearMotionInstant goal{target_position, rj_geometry::Point{0.0, 0.0}};
    
    intent.motion_command = planning::MotionCommand{"path_target", goal, face_option, false};
    return intent;
}
}  // namespace strategy