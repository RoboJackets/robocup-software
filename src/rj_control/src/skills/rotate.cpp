#include "rj_control/skills/rotate.hpp"

namespace control {

ControlCommand Rotate::update(
    int robot_id,
    const WorldState& world_state,
    const double& heading,
    [[maybe_unused]] Pid& x_controller,
    [[maybe_unused]] Pid& y_controller,
    Pid& w_controller
) {
    const double& current_heading = world_state.get_robot(true, robot_id).pose.heading();

    return {{
        0.0,
        w_controller.update(heading, current_heading)
    }};
}

bool Rotate::complete(
    int robot_id,
    const WorldState& world_state,
    const double& heading, //NOLINT(bugprone-easily-swappable-parameters)
    const double& tolerance
) {
    const double& current_heading = world_state.get_robot(true, robot_id).pose.heading();

    double angular_difference = heading - current_heading;
    double difference = std::atan2(std::sin(angular_difference), std::cos(angular_difference));
    return std::abs(difference) < tolerance;
}

} // namespace control