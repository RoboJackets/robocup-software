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

    if (robot_id_ == goalie_id_) {
        set_current_position<Goalie>();
        return current_position_->get_task(*last_world_state_, field_dimensions_,
                                           current_play_state_);
    }

    // Update our state
    process_play_state();

    update_position();

    return current_position_->get_task(*last_world_state_, field_dimensions_, current_play_state_);
}

void RobotFactoryPosition::process_play_state() {

    // UPDATE THIS TO INSTEAD BE LIKE IF RESTART CHANGED 
    // AND THEN SEPARTE FOR IF OTHER STATE CHANGED

    // Check if play states are ==, ignoring the ball placement position.
    if (last_play_state_.same_as(current_play_state_)) {
        // Nothing has changed. Continue as usual.
        return;
    }

    // The referee/GC has changed states.
    last_play_state_ = current_play_state_;

    if (current_play_state_.is_their_restart()) {
        // This is the other team's restart!

        switch (current_play_state_.restart()) {
            case PlayState::Restart::Free:
            case PlayState::Restart::Kickoff: {
                // In a kick for the other team.
                // We want to change as little as possible, because once they
                // kick the ball, everything should go back to how it just was.

                // TODO(Sid): Ideally, we actually do nothing here.
                // Offense and defense should just be aware of this state, and
                // not touch the ball.

                // The only rule we have to follow right now is to not touch the
                // ball until the other team does. Once the other team touches the
                // ball, the GC will advance past this state.

                if (current_position_->get_name() == "Offense" ||
                    current_position_->get_name() == "PenaltyPlayer" ||
                    current_position_->get_name() == "FreeKicker") {
                    // If we are playing a position that might illegally touch the ball,
                    // just set ourselves to idle for the time being.
                    set_current_position<Idle>();
                }
                break;
            }

            case PlayState::Restart::Placement: {
                // If the other team is doing a ball placement
                // We should be able to play as normal
                // Ball placement simply adds an obstacle
                // (get out of the way of the placement line)
                // Which we can/will add to the static obstacles
                // as necessary
                // TODO(https://app.clickup.com/t/86azm4y6z)

                set_default_positions();
                break;
            }

            case PlayState::Restart::Penalty: {
                // If the other team is kicking a penalty kick,
                // a number of new rules apply

                // If we are the goalie, we need to stay on the baseline
                // until play has started

                // If we are any other robot, we must stay 0.5 meters behind the
                // ball at all times

                // TODO(https://app.clickup.com/t/86azm51j4) assign positions that
                // follow these rules
                set_current_position<Idle>();
                break;
            }

            case PlayState::Restart::None: {
                // Should not happen!
                break;
            }
        }
    } else {
        switch (current_play_state_.restart()) {
            case PlayState::Restart::Free:
            case PlayState::Restart::Kickoff:
            case PlayState::Restart::Penalty: {
                // We have just been assigned (on this tick) some restart.
                // We must first choose which robot should kick.
                choose_kicker();
                break;
            }

            case PlayState::Restart::Placement: {
                // We currently do not perform ball placement.
                // This should never happen.

                set_current_position<Idle>();
                break;
            }

            case PlayState::Restart::None: {
                // There is no restart.
                set_default_positions();
                break;
            }
        }
    }
}


void RobotFactoryPosition::update_position() {

    auto alive_bot_count = std::count(alive_robots_.begin(), alive_robots_.end(), true);

    if (current_play_state_.is_our_restart()) {
        // Idle until a kicker is chosen

        if (kicker_distances_.size() < alive_bot_count - 1) {
            // No kicker chosen yet
            set_current_position<Idle>();
            return;
        }

        // A kicker has been chosen.
        bool is_kicker = (get_closest_kicker(kicker_distances_).first == robot_id_);

        // Figure out what we need to do

    }
}





    switch (current_state_) {
        case FREE_KICK: {
            if (kicker_distances_.count(robot_id_) == 0) {
                SPDLOG_INFO("Robot {} sent a kick request", robot_id_);
                broadcast_kicker_request();
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
            } else if (kicker_distances_.size() == 5) {
                if (get_closest_kicker(kicker_distances_).first == robot_id_) {
                    set_current_position<Defense>();
                } else {
                    set_current_position<Defense>();
                }
            } else {
                set_current_position<Idle>();
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
            } else if (kicker_distances_.size() == 5) {
                if (get_closest_kicker(kicker_distances_).first == robot_id_) {
                    set_current_position<PenaltyPlayer>();
                } else {
                    set_current_position<Defense>();
                }
            } else {
                set_current_position<Idle>();
            }
            break;
        }

        case KICKOFF_KICK: {
            set_current_position<PenaltyPlayer>();
            break;
        }

        case STOP: {
            // if (current_position_->get_name() != "Defense") {
            //     current_position_ = std::make_unique<Defense>(robot_id_);
            // }
            set_default_positions(*last_world_state_, field_dimensions_);
            SPDLOG_INFO("Robot {} Speed: {}", robot_id_,
                        last_world_state_->get_robot(true, robot_id_).velocity.linear().mag());
            break;
        }

        case PLAYING: {
            set_default_positions(*last_world_state_, field_dimensions_);
            break;
        }
    }
}

void RobotFactoryPosition::choose_kicker() {
    kicker_distances_.clear();
    broadcast_kicker_request();
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

void RobotFactoryPosition::set_default_positions() {
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
        if (i <= 1) {
            set_current_position<Defense>();
        } else {
            set_current_position<Offense>();
        }
    } else {
        // Defensive mode
        // Closest 4 robots on defense, rest on offense
        if (i <= 3) {
            set_current_position<Defense>();
        } else {
            set_current_position<Offense>();
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
