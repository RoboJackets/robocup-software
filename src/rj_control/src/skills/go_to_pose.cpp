#include "rj_control/skills/go_to_pose.hpp"

namespace control {

ControlCommand GoToPose::update(
    int robot_id,
    const WorldState& world_state,
    const rj_geometry::Pose& command,
    Pid& x_controller,
    Pid& y_controller,
    Pid& w_controller
) {
    return {{
        x_controller.update(command.position().x(), world_state.get_robot(true, robot_id).pose.position().x()),
        y_controller.update(command.position().y(), world_state.get_robot(true, robot_id).pose.position().y()),
        w_controller.update(command.heading(), world_state.get_robot(true, robot_id).pose.heading())
    }};
}

bool GoToPose::complete(
    int robot_id,
    const WorldState& world_state,
    const rj_geometry::Pose& command,
    const std::pair<double, double>& tolerance
) {
    const rj_geometry::Pose& state = world_state.get_robot(true, robot_id).pose;

    double position_error = (state.position() - command.position()).mag();
    double angular_error = (state.heading() - command.heading());
    return position_error <= tolerance.first &&
        angular_error <= tolerance.second;
}

} // namespace control