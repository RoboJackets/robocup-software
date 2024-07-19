#include "robot_factory_position.hpp"

#include <algorithm>

#include "idle.hpp"
#include "penalty_non_kicker.hpp"

namespace strategy {

RobotFactoryPosition::RobotFactoryPosition(int r_id) : Position(r_id, "RobotFactoryPosition") {
    if (robot_id_ == goalie_id_) {
        current_position_ = std::make_unique<Goalie>(robot_id_);
    } else if (robot_id_ == 0 || robot_id_ == 0 || robot_id_ == solo_offense_id_) {
        current_position_ = std::make_unique<SoloOffense>(robot_id_);
    } else {
        current_position_ = std::make_unique<Defense>(robot_id_);
    }
}

std::optional<RobotIntent> RobotFactoryPosition::derived_get_task([
    [maybe_unused]] RobotIntent intent) {
    if (robot_id_ == goalie_id_) {
        set_current_position<Goalie>();
        return current_position_->get_task(*last_world_state_, field_dimensions_,
                                           current_play_state_);
    }

    // Update our state
    process_play_state();

    // Every tick, update position based on PlayState
    update_position();

    return current_position_->get_task(*last_world_state_, field_dimensions_, current_play_state_);
}

void RobotFactoryPosition::process_play_state() {
    // UPDATE THIS TO INSTEAD BE LIKE IF RESTART CHANGED
    // AND THEN SEPARTE FOR IF OTHER STATE CHANGED

    if (last_play_state_.state() != current_play_state_.state()) {
        last_play_state_ = current_play_state_;
        SPDLOG_INFO("STATE: {}, RESTART: {}", current_play_state_.state(),
                    current_play_state_.restart());
        switch (current_play_state_.state()) {
            case PlayState::State::Playing: {
                // We just became regular playing.
                // set_default_position();
                break;
            }

            case PlayState::State::Setup: {
                // We just entered the setup phase of either Kickoff or Penalty Kick
                handle_setup();
                break;
            }

            case PlayState::State::Ready: {
                // We entered the ready (kicking) phase of either a kickoff, penalty kick, OR free
                // kick
                handle_ready();
                break;
            }

            case PlayState::State::PenaltyPlaying: {
                // We entered the penalty playing phase. Only the goalie and striker should be
                // moving.

                // TODO(https://app.clickup.com/t/86azm51j4) we should handle this at a lower level
                handle_penalty_playing();
                break;
            }

            case PlayState::State::Stop:
            case PlayState::State::Halt: {
                // The game has been stopped or halted. In this case, we typically want to keep
                // our current position. The rules for movement should be handled at a lower level.
                handle_stop();
                break;
            }
        }
    }
}

void RobotFactoryPosition::handle_stop() { set_default_position(); }
void RobotFactoryPosition::handle_penalty_playing() {
    if (!am_closest_kicker()) {
        set_current_position<SmartIdle>();
    }
}

void RobotFactoryPosition::handle_setup() {
    // Set up some restart
    if (current_play_state_.is_our_restart()) {
        // Set up our restart

        if (current_play_state_.is_kickoff() || current_play_state_.is_penalty()) {
            start_kicker_picker();
        } else {
            SPDLOG_WARN("Invalid restart setup!");
        }
    }
}

void RobotFactoryPosition::handle_ready() {
    // Ready stage for a restart
    // Time to kick

    if (current_play_state_.is_our_restart() && current_play_state_.is_free_kick()) {
        // There is no "Setup" stage for free kicks, so this is when we choose kicker
        start_kicker_picker();

    } else if (current_play_state_.is_their_restart() && current_play_state_.is_free_kick()) {
        if (current_position_->get_name() == "Offense" ||
            current_position_->get_name() == "PenaltyPlayer" ||
            current_position_->get_name() == "SoloOffense" ||
            current_position_->get_name() == "GoalKicker") {
            set_current_position<Idle>();
        }
    }
}

void RobotFactoryPosition::update_position() {
    // set_current_position<Line>();
    // return;
    switch (current_play_state_.state()) {
        case PlayState::State::Playing: {
            // We just became regular playing.
            set_default_position();
            break;
        }

        case PlayState::State::Setup:
        case PlayState::State::Ready: {
            // Currently in setup

            // This is the only case where we have to do something on every tick
            if (current_play_state_.is_our_restart()) {
                if (have_all_kicker_responses()) {
                    if (am_closest_kicker()) {
                        if (current_play_state_.is_free_kick() ||
                            current_play_state_.is_kickoff()) {
                            set_current_position<FreeKicker>();
                        } else {
                            set_current_position<PenaltyPlayer>();
                        }
                    } else {
                        if (current_play_state_.is_kickoff()) {
                            set_current_position<Defense>();
                        } else if (current_play_state_.is_penalty()) {
                            // set_current_position<SmartIdle>();
                            set_current_position<PenaltyNonKicker>();
                        } else if (current_play_state_.is_free_kick()) {
                            // do what it was doing before foul
                            set_default_position();
                            // don't want a player on offense to try to kick the
                            // ball instead of free kicker
                            if (current_position_->get_name() == "Offense") {
                                set_current_position<SmartIdle>();
                            }
                        }
                    }
                } else {
                    set_current_position<SmartIdle>();
                }

            } else {  // Their restart
                if (current_play_state_.is_kickoff()) {
                    set_current_position<Defense>();
                } else if (current_play_state_.is_penalty()) {
                    // set_current_position<SmartIdle>();
                    set_current_position<PenaltyNonKicker>();
                } else if (current_play_state_.is_free_kick()) {
                    // do what it was doing before foul
                    set_default_position();
                    // don't want a player on offense to try to kick the
                    // ball instead of free kicker
                    if (current_position_->get_name() == "Offense") {
                        set_current_position<SmartIdle>();
                    }
                }
            }

            break;
        }

        case PlayState::State::PenaltyPlaying:
            if (am_closest_kicker()) {
                if (current_play_state_.is_free_kick()) {
                    set_current_position<FreeKicker>();
                } else {
                    set_current_position<PenaltyPlayer>();
                }
            }
            break;
        case PlayState::State::Stop:
        case PlayState::State::Halt: {
            // No action needed on each tick
            break;
        }
    }
}

void RobotFactoryPosition::start_kicker_picker() {
    return;
    // SPDLOG_INFO("robot {} has cleared", robot_id_);
    kicker_distances_.clear();
    broadcast_kicker_request();
}

bool RobotFactoryPosition::have_all_kicker_responses() {
    return true;
    int num_alive = std::count(alive_robots_.begin(), alive_robots_.end(), true);

    return kicker_distances_.size() == num_alive - 1;  // Don't expect the goalie to respond
}

bool RobotFactoryPosition::am_closest_kicker() {
    return robot_id_ == solo_offense_id_;
    // Return the max, comparing by distances only
    auto closest =
        std::min_element(kicker_distances_.begin(), kicker_distances_.end(),
                         [](const std::pair<int, double>& a, const std::pair<int, double>& b) {
                             if (a.second == b.second) {
                                 return a.first < b.first;
                             }
                             return a.second < b.second;
                         });

    // Closest is an iterator to the pair (robot_id, distance)
    return closest->first == robot_id_;
}

void RobotFactoryPosition::set_default_position() {
    // zoner defense testing

    if (robot_id_ == goalie_id_) {
        return;
    }
    if (robot_id_ == 0 || robot_id_ == 0) {
        set_current_position<SoloOffense>();
    } else if (robot_id_ == solo_offense_id_) {
        set_current_position<SoloOffense>();
    } else {
        set_current_position<Defense>();
    }
    return;
    // end zoner defense testing

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
