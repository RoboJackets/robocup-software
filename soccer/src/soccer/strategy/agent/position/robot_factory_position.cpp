#include "robot_factory_position.hpp"

#include "strategy/agent/position/line.hpp"

namespace strategy {

RobotFactoryPosition::RobotFactoryPosition(int r_id) : Position(r_id, "RobotFactoryPosition") {
    current_position_ = std::make_unique<Line>(robot_id_);

    // if (robot_id_ == 0) {
    //     current_position_ = std::make_unique<Goalie>(robot_id_);
    // } else if (robot_id_ == 1 || robot_id_ == 2) {
    //     // } else if (robot_id_ == 1) {
    //     current_position_ = std::make_unique<Offense>(robot_id_);
    // } else {
    //     current_position_ = std::make_unique<Defense>(robot_id_);
    // }
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
    for (int i = 0; i < static_cast<int>(kNumShells); i++) {
        // Ignore goalie
        if (i == goalie_id_) {
            continue;
        }
        if (alive_robots_[i]) {
            robots_copy.emplace_back(i, last_world_state_->our_robots[i].pose.position().y());
        }
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
    if (our_possession_ || last_world_state_->ball.position.y() >
                               field_dimensions_.center_field_loc().y() - kBallDiameter) {
        // Offensive mode
        // Closest 2 robots on defense, rest on offense
        if (i <= 3) {
            set_current_position<Defense>();
        } else {
            set_current_position<SoloOffense>();
        }
    } else {
        // Defensive mode
        // Closest 4 robots on defense, rest on offense
        if (i <= 3) {
            set_current_position<Defense>();
        } else {
            set_current_position<SoloOffense>();
        }
    }
}

std::deque<communication::PosAgentRequestWrapper>
RobotFactoryPosition::send_communication_request() {
    // Return both this position's communication requests and its child position's communication
    // requests

    // This class
    auto result = Position::send_communication_request();

    // Delegated class
    auto current = current_position_->send_communication_request();

    // Combine the two
    result.insert(result.end(), current.begin(), current.end());

    return result;
}

void RobotFactoryPosition::receive_communication_response(
    communication::AgentPosResponseWrapper response) {
    // Call to super
    current_position_->receive_communication_response(response);
}

communication::PosAgentResponseWrapper RobotFactoryPosition::receive_communication_request(
    communication::AgentPosRequestWrapper request) {
    if (const communication::KickerRequest* kicker_request =
            std::get_if<communication::KickerRequest>(&request.request)) {
        if (kicker_distances_.size() >= 1 && !have_all_kicker_responses() &&
            current_position_->get_name() != "GoalKicker") {
            bool prev = kicker_distances_.count(kicker_request->robot_id) >= 1;
            if (!prev) {
                kicker_distances_[kicker_request->robot_id] = kicker_request->distance;
                broadcast_kicker_request();
            }
        }
    }
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

std::string RobotFactoryPosition::get_current_state() {
    return current_position_->get_current_state();
}

}  // namespace strategy
