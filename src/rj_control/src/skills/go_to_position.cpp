#include "rj_control/skills/go_to_position.hpp"

namespace control {

ControlCommand GoToPosition::update(
    int robot_id,
    const WorldState& world_state,
    const rj_geometry::Point& command,
    Pid& x_controller,
    Pid& y_controller,
    [[maybe_unused]] Pid& w_controller
) {
    return {{
        x_controller.update(command.x(), world_state.get_robot(true, robot_id).pose.position().x()),
        y_controller.update(command.y(), world_state.get_robot(true, robot_id).pose.position().y()),
        0.0
    }};
}

bool GoToPosition::complete(
    int robot_id,
    const WorldState& world_state,
    const rj_geometry::Point& command,
    const double& tolerance
) {
    const rj_geometry::Point& state = world_state.get_robot(true, robot_id).pose.position();
    return (state - command).mag() <= tolerance;
}

} // namespace control