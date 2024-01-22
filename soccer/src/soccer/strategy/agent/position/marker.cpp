#include "marker.hpp"

namespace strategy {
Marker::Marker(u_int8_t robot_id) {
    this->target_robot_id = robot_id;
}

std::optional<RobotIntent> Marker::get_task(RobotIntent intent,
                                            const WorldState* world_state, FieldDimensions field_dimensions) {
    rj_geometry::Point targetPoint = 
        world_state->get_robot(false, 4).pose.position();
    rj_geometry::Point ballPoint = 
        world_state->ball.position;
    rj_geometry::Segment ballToTarget = rj_geometry::Segment(targetPoint, ballPoint);
    rj_geometry::Segment ballToTargetBiased = 
        rj_geometry::Segment(ballToTarget.center(), targetPoint);
    planning::LinearMotionInstant goal{ballToTargetBiased.center(), rj_geometry::Point{0.0, 0.0}};
    intent.motion_command = planning::MotionCommand{"path_target", goal, planning::FaceBall{}, true};
    return intent;
}
}  // namespace strategy