#include "robot_factory_position.hpp"

namespace strategy {

RobotFactoryPosition::RobotFactoryPosition(int r_id) : Position(r_id) {
    position_name_ = "RobotFactoryPosition";

    if (robot_id_ == 0) {
        current_position_ = std::make_unique<Goalie>(robot_id_);
    } else if (robot_id_ == 1) {
        current_position_ = std::make_unique<Offense>(robot_id_);
    } else {
        current_position_ = std::make_unique<Defense>(robot_id_);
    }
}

std::optional<RobotIntent> RobotFactoryPosition::get_task(WorldState& world_state,
                                                          FieldDimensions& field_dimensions) {
    // If keeper, make no changes
    if (robot_id_ == 0) {
        return current_position_->get_task(world_state, field_dimensions);
    }

    // TODO (Rishi and Jack): Make this synchronized across all robots to avoid race conditions

    // Get sorted positions of all friendly robots
    using RobotPos = std::pair<int, double>;  // (robotId, yPosition)

    std::vector<RobotPos> robots_copy;
    for (int i = 0; i < 6; i++) {
        // Ignore goalie
        if (i == 0) {
            continue;
        }
        robots_copy.emplace_back(i, world_state.our_robots[i].pose.position().y());
    }

    std::sort(robots_copy.begin(), robots_copy.end(),
              [](RobotPos const& a, RobotPos const& b) { return a.second < b.second; });

    // Find relative location of current robot
    int i = 0;
    for (RobotPos r : robots_copy) {
        if (r.first == robot_id_) {
            break;
        }
        i++;
    }

    // Assigning new position
    // Checking whether we have possesion or if the ball is on their half (using 1.99 to avoid
    // rounding issues on midline)
    if (Position::our_possession_ ||
        world_state.ball.position.y() > field_dimensions.length() / 1.99) {
        // Offensive mode
        // Closest 2 robots on defense, rest on offense
        if (i <= 1) {
            if (current_position_->get_name() != "Defense") {
                current_position_ = std::make_unique<Defense>(robot_id_);
            }
        } else {
            if (current_position_->get_name() != "Offense") {
                current_position_ = std::make_unique<Offense>(robot_id_);
            }
        }
    } else {
        // Defensive mode
        // Closest 4 robots on defense, rest on offense
        if (i <= 3) {
            if (current_position_->get_name() != "Defense") {
                current_position_ = std::make_unique<Defense>(robot_id_);
            }
        } else {
            if (current_position_->get_name() != "Offense") {
                current_position_ = std::make_unique<Offense>(robot_id_);
            }
        }
    }

    return current_position_->get_task(world_state, field_dimensions);
}

std::optional<RobotIntent> RobotFactoryPosition::derived_get_task(
    [[maybe_unused]] RobotIntent intent) {
    SPDLOG_ERROR("RobotFactory derived_get_task() should not be called!");
    return std::nullopt;
}

std::optional<communication::PosAgentRequestWrapper>
RobotFactoryPosition::send_communication_request() {
    // Call to super
    return current_position_->send_communication_request();
}

void RobotFactoryPosition::receive_communication_response(
    communication::AgentPosResponseWrapper response) {
    // Call to super
    current_position_->receive_communication_response(response);
}

communication::PosAgentResponseWrapper RobotFactoryPosition::receive_communication_request(
    communication::AgentPosRequestWrapper request) {
    // Return the response
    return current_position_->receive_communication_request(request);
}

void RobotFactoryPosition::derived_acknowledge_pass() {
    current_position_->derived_acknowledge_pass();
}

void RobotFactoryPosition::derived_pass_ball() { current_position_->derived_pass_ball(); }

void RobotFactoryPosition::derived_acknowledge_ball_in_transit() {
    current_position_->derived_acknowledge_ball_in_transit();
}

void RobotFactoryPosition::set_is_done() { current_position_->set_is_done(); }

void RobotFactoryPosition::die() { current_position_->die(); }

void RobotFactoryPosition::revive() { current_position_->revive(); }

std::string RobotFactoryPosition::get_current_state() { return current_position_->get_current_state(); }

}  // namespace strategy
