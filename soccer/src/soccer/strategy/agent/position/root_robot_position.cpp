#include "root_robot_position.hpp"

namespace strategy {

RootRobotPosition::RootRobotPosition(int r_id) : Position(r_id) { position_name_ = "RootRobotPosition";

if (robot_id_ == 0) {
        current_position_ = std::make_unique<Goalie>(robot_id_);
    } else if (robot_id_ == 1) {
        current_position_ = std::make_unique<Offense>(robot_id_);
    } else {
        current_position_ = std::make_unique<Defense>(robot_id_);
    }

}

std::optional<RobotIntent> RootRobotPosition::derived_get_task(RobotIntent intent) {
    // This is where the current_position can be reassigned based on the PlayState
    return current_position_->get_task(*last_world_state_, field_dimensions_);
}

void RootRobotPosition::receive_communication_response(communication::AgentPosResponseWrapper response) {
    // Call to super
    current_position_->receive_communication_response(response);

}

communication::PosAgentResponseWrapper RootRobotPosition::receive_communication_request(
    communication::AgentPosRequestWrapper request) {

    // Return the response
    return current_position_->receive_communication_request(request);
}

void RootRobotPosition::derived_acknowledge_pass() {
    current_position_->derived_acknowledge_pass();
}

void RootRobotPosition::derived_pass_ball() {
    current_position_->derived_pass_ball();
}

void RootRobotPosition::derived_acknowledge_ball_in_transit() {
    current_position_->derived_acknowledge_ball_in_transit();
}

void RootRobotPosition::die() {
    current_position_->die();
}

void RootRobotPosition::revive() {
    current_position_->revive();
}

}  // namespace strategy
