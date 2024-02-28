#include "robot_factory_position.hpp"

#include <algorithm>

#include "idle.hpp"

namespace strategy {

RobotFactoryPosition::RobotFactoryPosition(int r_id) : Position(r_id, "RobotFactoryPosition") {
    if (robot_id_ == 0) {
        current_position_ = std::make_unique<Goalie>(robot_id_);
    } else if (robot_id_ == 1 || robot_id_ == 2) {
        current_position_ = std::make_unique<Offense>(robot_id_);
    } else {
        current_position_ = std::make_unique<Defense>(robot_id_);
    }
}

std::optional<RobotIntent> RobotFactoryPosition::derived_get_task(
    [[maybe_unused]] RobotIntent intent) {
    // SPDLOG_INFO("Robot {} size {}", robot_id_, kicker_distances_.size());
    // SPDLOG_INFO("Free Kick {}", current_play_state_.is_free_kick());

    if (robot_id_ == goalie_id_) {
        if (current_position_->get_name() != "Goalie") {
            current_position_ = std::make_unique<Goalie>(robot_id_);
        }
        return current_position_->get_task(*last_world_state_, field_dimensions_,
                                           current_play_state_);
    }    

    current_state_ = update_state();

    state_to_task();

    return current_position_->get_task(*last_world_state_, field_dimensions_, current_play_state_);
}

RobotFactoryPosition::State RobotFactoryPosition::update_state() {
    RobotFactoryPosition::State next_state = current_state_;
    switch (current_state_) {
        case FREE_KICK: {
            if (!current_play_state_.is_free_kick()) {
                kicker_distances_.clear();
                next_state = PLAYING;
            }
            break;
        } case PENALTY_SETUP: {
            if (current_play_state_.is_penalty() && current_play_state_.is_ready()) {
                next_state = PENALTY_KICK;
            }
            break;
        } case PENALTY_KICK: {
            if (!current_play_state_.is_penalty()) {
                kicker_distances_.clear();
                next_state = PLAYING;
            }
            break;
        } case KICKOFF_SETUP: {
            if (current_play_state_.is_kickoff() && current_play_state_.is_ready()) {
                next_state = KICKOFF_KICK;
            }
            break;
        } case KICKOFF_KICK: {
            if (!current_play_state_.is_kickoff()) {
                kicker_distances_.clear();
                next_state = PLAYING;
            }
            break;
        } case STOP: {
            if (!current_play_state_.is_stop()) {
                if (current_play_state_.is_free_kick()) {
                    next_state = FREE_KICK;
                } else if (current_play_state_.is_penalty() && current_play_state_.is_setup()) {
                    next_state = PENALTY_SETUP;
                } else if (current_play_state_.is_kickoff() && current_play_state_.is_setup()) {
                    next_state = KICKOFF_SETUP;
                } else {
                    next_state = PLAYING;
                }
            }
            break;
        }case PLAYING: {
            if (current_play_state_.is_free_kick()) {
                next_state = FREE_KICK;
            } else if (current_play_state_.is_penalty() && current_play_state_.is_setup()) {
                next_state = PENALTY_SETUP;
            } else if (current_play_state_.is_kickoff() && current_play_state_.is_setup()) {
                next_state = KICKOFF_SETUP;
            } else if (current_play_state_.is_stop()) {
                next_state = STOP;
            }
            break;
        }
    }
    return next_state;
}


void RobotFactoryPosition::state_to_task() {
    switch (current_state_) {
        case FREE_KICK: {
            std::string debug{};
            for (const auto& elem : kicker_distances_) {
                debug += std::to_string(elem.first) + "," + std::to_string(elem.second) + " ";
            }
            // SPDLOG_INFO("Robot {} map is {}", robot_id_, debug);

            if (kicker_distances_.count(robot_id_) == 0) {
                SPDLOG_INFO("Robot {} sent a kick request", robot_id_);
                broadcast_kicker_request();
                // SPDLOG_INFO("Robot {} count in factory {}", robot_id_,
                //             kicker_distances_.count(robot_id_));

            } else if (kicker_distances_.size() == 5) {
                if (get_closest_kicker(kicker_distances_).first == robot_id_) {
                    if (current_position_->get_name() != "FreeKicker") {
                        SPDLOG_INFO("Robot {} is chosen as free kicker", robot_id_);

                        current_position_ = std::make_unique<FreeKicker>(robot_id_);
                    }

                } else {
                    if (current_position_->get_name() != "Defense") {
                        SPDLOG_INFO("Robot {} is not chosen as free kicker", robot_id_);

                        current_position_ = std::make_unique<Defense>(robot_id_);
                    }
                }
            } else {
                if (current_position_->get_name() != "Idle") {
                    current_position_ = std::make_unique<Idle>(robot_id_);
                }
            }
            break;
        }
        case PENALTY_SETUP: {
            if (kicker_distances_.count(robot_id_) == 0) {
                SPDLOG_INFO("Robot {} sent a kick request", robot_id_);
                broadcast_kicker_request();
                // SPDLOG_INFO("Robot {} count in factory {}", robot_id_,
                //             kicker_distances_.count(robot_id_));

            } else if (kicker_distances_.size() == 5) {
                if (get_closest_kicker(kicker_distances_).first == robot_id_) {
                    if (current_position_->get_name() != "PenaltyPlayer") {
                        SPDLOG_INFO("Robot {} is chosen as penalty kicker", robot_id_);

                        current_position_ = std::make_unique<PenaltyPlayer>(robot_id_);
                    }

                } else {
                    if (current_position_->get_name() != "Defense") {
                        SPDLOG_INFO("Robot {} is not chosen as penalty kicker", robot_id_);

                        current_position_ = std::make_unique<Defense>(robot_id_);
                    }
                }
            } else {
                if (current_position_->get_name() != "Idle") {
                    current_position_ = std::make_unique<Idle>(robot_id_);
                }
            }
            break;
        }
        case PENALTY_KICK: {
            if (current_position_->get_name() == "PenaltyPlayer") {
                current_position_ = std::make_unique<GoalKicker>(robot_id_);
            }
            break;
        }
        case KICKOFF_SETUP: {
            if (kicker_distances_.count(robot_id_) == 0) {
                SPDLOG_INFO("Robot {} sent a kick request", robot_id_);
                broadcast_kicker_request();
                // SPDLOG_INFO("Robot {} count in factory {}", robot_id_,
                //             kicker_distances_.count(robot_id_));

            } else if (kicker_distances_.size() == 5) {
                if (get_closest_kicker(kicker_distances_).first == robot_id_) {
                    if (current_position_->get_name() != "PenaltyPlayer") {
                        SPDLOG_INFO("Robot {} is chosen as Kickoff kicker", robot_id_);

                        current_position_ = std::make_unique<PenaltyPlayer>(robot_id_);
                    }

                } else {
                    if (current_position_->get_name() != "Defense") {
                        SPDLOG_INFO("Robot {} is not chosen as penalty kicker", robot_id_);

                        current_position_ = std::make_unique<Defense>(robot_id_);
                    }
                }
            } else {
                if (current_position_->get_name() != "Idle") {
                    current_position_ = std::make_unique<Idle>(robot_id_);
                }
            }
            break;
        }
        case KICKOFF_KICK: {
            if (current_position_->get_name() == "PenaltyPlayer") {
                current_position_ = std::make_unique<FreeKicker>(robot_id_);
            }
            break;
        }
        case STOP: {
            // if (current_position_->get_name() != "Defense") {
            //     current_position_ = std::make_unique<Defense>(robot_id_);
            // }
            set_default_positions(*last_world_state_, field_dimensions_);
            SPDLOG_INFO("Robot {} Speed: {}", robot_id_, last_world_state_->get_robot(true, robot_id_).velocity.linear().mag());
            break;
        }
        case PLAYING: {
            set_default_positions(*last_world_state_, field_dimensions_);
            break;
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
        kicker_distances_[kicker_request->robot_id] = kicker_request->distance;

        SPDLOG_INFO("Robot {} recieved a kick request from {}", robot_id_,
                    kicker_request->robot_id);
    }
    // Return the response
    return current_position_->receive_communication_request(request);
}

std::pair<int, double> RobotFactoryPosition::get_closest_kicker(
    const std::unordered_map<int, double>& kicker_distances) {
    // Return the max, comparing by distances only
    return *std::min_element(kicker_distances.begin(), kicker_distances.end(),
                             [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
                                 if (a.second == b.second) {
                                     return a.first < b.first;
                                 }
                                 return a.second < b.second;
                             });
}

void RobotFactoryPosition::set_default_positions(WorldState& world_state,
                                                 FieldDimensions& field_dimensions) {
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
            robots_copy.emplace_back(i, world_state.our_robots[i].pose.position().y());
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
    if (our_possession_ ||
        world_state.ball.position.y() > field_dimensions.center_field_loc().y() - kBallDiameter) {
        // Offensive mode
        // Closest 2 robots on defense, rest on offense
        if (i <= 1) {
            if (current_position_->get_name() != "Defense") {
                current_position_ = std::make_unique<Defense>(*current_position_);
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
                current_position_ = std::make_unique<Offense>(*current_position_);
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

std::queue<communication::PosAgentRequestWrapper>
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
    if (const communication::KickerRequest* kicker_request =
                   std::get_if<communication::KickerRequest>(&request.request)) {
        current_position_->kicker_distances_[kicker_request->robot_id] = kicker_request->distance;

        //TODO: Edit this when Alive Robots exists
        if (current_position_->kicker_distances_.size() == 6) {
            std::pair<int, double> closest_kicker = get_closest_kicker(current_position_->kicker_distances_);
            current_position_->is_kicker_ = (closest_kicker.first == robot_id_);
            current_position_->kicker_distances_.clear();
        }
    }
    // Return the response
    return current_position_->receive_communication_request(request);
}


std::pair<int, double> RobotFactoryPosition::get_closest_kicker(std::unordered_map<int, double> kicker_distances) {
    std::pair<int, double> closest_kicker = {-1, 10000};
    for (const auto & [key, value] : kicker_distances) {
        if (value < closest_kicker.second) {
            closest_kicker = {key, value};
        }
    }
    return closest_kicker;
}

void RobotFactoryPosition::set_default_positions(WorldState& world_state, FieldDimensions& field_dimensions) {
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
            robots_copy.emplace_back(i, world_state.our_robots[i].pose.position().y());
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
    if (our_possession_ ||
        world_state.ball.position.y() > field_dimensions.center_field_loc().y() - kBallDiameter) {
        // Offensive mode
        // Closest 2 robots on defense, rest on offense
        if (i <= 1) {
            if (current_position_->get_name() != "Defense") {
                current_position_ = std::make_unique<Defense>(*current_position_);
            }
        } else {
            if (current_position_->get_name() != "Offense") {
                current_position_ = std::make_unique<Offense>(*current_position_);
            }
        }
    } else {
        // Defensive mode
        // Closest 4 robots on defense, rest on offense
        if (i <= 3) {
            if (current_position_->get_name() != "Defense") {
                current_position_ = std::make_unique<Defense>(*current_position_);
            }
        } else {
            if (current_position_->get_name() != "Offense") {
                current_position_ = std::make_unique<Offense>(*current_position_);
            }
        }
    }
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
