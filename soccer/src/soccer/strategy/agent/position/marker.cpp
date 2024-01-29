#include "marker.hpp"

namespace strategy {
Marker::Marker(u_int8_t robot_id) {
}

std::optional<RobotIntent> Marker::get_task(RobotIntent intent,
    const WorldState* world_state, FieldDimensions field_dimensions) {
    rj_geometry::Point targetPoint = 
        world_state->get_robot(false, target).pose.position();
    rj_geometry::Point ballPoint = 
        world_state->ball.position;
    rj_geometry::Point targetToBall = (ballPoint - targetPoint).normalized(0.55f); 
    planning::LinearMotionInstant goal{targetPoint + targetToBall, rj_geometry::Point{0.0, 0.0}};
    intent.motion_command = planning::MotionCommand{"path_target", goal, planning::FaceBall{}, true};
    return intent;
}

int Marker::choose_target(WorldState* ws) {

    for (int i = 0; i < 11; i++) {
        if (std::fabs(ws->get_robot(false, i).pose.position().x()) < 2.5
            && ws->get_robot(false, i).pose.position().y() < Y_BOUND
            && (ws->ball.position - ws->get_robot(false, i).pose.position()).mag() > .25) {
                target = i;
                return i;
        }
    }
    target = -1;
    return -1;
}

bool Marker::target_out_of_bounds(WorldState* ws) {
    if (target == -1) return true;
    if (ws->get_robot(false, target).pose.position().y() > Y_BOUND) {
        target = -1;
        return true;
    }
    return false;
}

int Marker::get_target() {
    return target;
}
}  // namespace strategy