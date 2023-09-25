#include "runner.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position(r_id) {
    position_name_ = "Runner"
    WorldState* world_state = this->world_state();
}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    latest_state_ = update_state();
    return state_to_task(intent);
}

Runner::State Runner::update_state() {
    WorldState* world_state = this->world_state();
    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
    if (latest_state_ == IDLING) {
        Runner::initial_position = robot_position;
        return RUNFIRSTSIDE;
    } else if (latest_state_ == RUNFIRSTSIDE) {
        if ((robot_position.y() - initial_position.y()) > Runner::side_length) {
            Runner::initial_position = robot_position;
            return RUNSECONDSIDE;
        }
    } else if (latest_state_ == RUNSECONDSIDE) {
        if ((initial_position.x() - robot_position.x()) < Runner::side_length) {
            Runner::initial_position = robot_position;
            return RUNTHIRDSIDE;
        }
    } else if (latest_state_ == RUNTHIRDSIDE) {
        if ((initial_position.y() - robot_position.y()) > Runner::side_length) {
            Runner::initial_position = robot_position;
            return RUNFOURTHSIDE;
        }
    } else if (latest_state_ == RUNFOURTHSIDE) {
        if ((robot_position.x() - initial_position.x() > Runner::side_length)) {
            Runner::initial_position = robot_position;
            return IDLING;
        }
    }
    return latest_state_;
}

std::optional<RobotIntent> state_to_task(RobotIntent intent) {
}


} // namespace strategy
