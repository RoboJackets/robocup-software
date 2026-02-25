#include "rj_strategy/agent/position/robot_factory_position.hpp"
#include "rj_strategy/agent/position/dribbler.hpp"

namespace strategy {

RobotFactoryPosition::RobotFactoryPosition(int r_id, rclcpp::Node::SharedPtr node)
    : Position(r_id, "RobotFactoryPosition"), kicker_picker_(std::move(node), r_id) {
    if (robot_id_ == 0) {
        // Robot 0 stays idle and doesn't move
        current_position_ = std::make_unique<Idle>(robot_id_);
    } else if (robot_id_ == 1) {
        // Robot 1 is always the Dribbler
        current_position_ = std::make_unique<Dribbler>(robot_id_);
    } else if (robot_id_ == 2) {
        // Robot 2 stays idle and doesn't move
        current_position_ = std::make_unique<Idle>(robot_id_);
    } else {
        // All other robots use SmartIdle for testing
        current_position_ = std::make_unique<SmartIdle>(robot_id_);
    }
}

std::optional<RobotIntent> RobotFactoryPosition::derived_get_task([
    [maybe_unused]] RobotIntent intent) {
    // Robots 0, 1, and 2 maintain their positions and don't change based on goalie_id
    // Robot 0 and 2 are Idle, Robot 1 is Dribbler
    if (robot_id_ == 0 || robot_id_ == 1 || robot_id_ == 2) {
        return current_position_->get_task(*last_world_state_, field_dimensions_,
                                           current_play_state_);
    }
    
    // For other robots, handle goalie logic
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
        switch (current_play_state_.state()) {
            case PlayState::State::Playing: {
                // We just became regular playing.
                // Robot 0, 1, and 2 maintain their positions (0: Idle, 1: Dribbler, 2: Idle)
                if (robot_id_ != 0 && robot_id_ != 1 && robot_id_ != 2) {
                    // set_default_position();
                    kicker_picker_.leave_group();
                }
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
                kicker_picker_.leave_group();
                handle_stop();
                break;
            }
        }
        last_play_state_ = current_play_state_;
    }
}

void RobotFactoryPosition::handle_stop() {
    // Robot 0, 1, and 2 maintain their positions even when stopped
    if (robot_id_ == 0 || robot_id_ == 1 || robot_id_ == 2) {
        return;
    }
    set_default_position();
}

void RobotFactoryPosition::handle_penalty_playing() {
    // Robot 0, 1, and 2 maintain their positions even during penalty playing
    if (robot_id_ == 0 || robot_id_ == 1 || robot_id_ == 2) {
        return;
    }
    if (!(kicker_picker_.am_i_member() && kicker_picker_.is_selected())) {
        set_current_position<SmartIdle>();
    }
}

void RobotFactoryPosition::handle_setup() {
    // Robot 0, 1, and 2 maintain their positions even during setup
    if (robot_id_ == 0 || robot_id_ == 1 || robot_id_ == 2) {
        return;
    }
    
    // Set up some restart
    if (current_play_state_.is_our_restart()) {
        // Set up our restart

        if ((current_play_state_.is_kickoff() || current_play_state_.is_penalty()) &&
            !kicker_picker_.am_i_member()) {
            kicker_picker_.join_group([this](KickerPickerClient::Result result) {
                if (result.am_i_member && result.kicker_id == robot_id_ &&
                    current_play_state_.is_kickoff()) {
                    set_current_position<FreeKicker>();
                } else if (result.am_i_member && result.kicker_id == robot_id_ &&
                           current_play_state_.is_penalty()) {
                    set_current_position<PenaltyPlayer>();
                } else if (current_play_state_.is_penalty()) {
                    set_current_position<PenaltyNonKicker>();
                } else if (current_play_state_.is_kickoff()) {
                    set_current_position<Defense>();
                }
            });
        } else {
            SPDLOG_WARN("Invalid restart setup!");
        }
    }
}

void RobotFactoryPosition::handle_ready() {
    // Robot 0, 1, and 2 maintain their positions even during ready phase
    if (robot_id_ == 0 || robot_id_ == 1 || robot_id_ == 2) {
        return;
    }
    
    // Ready stage for a restart
    // Time to kick

    if (current_play_state_.is_our_restart() && current_play_state_.is_free_kick() &&
        !kicker_picker_.am_i_member()) {
        // There is no "Setup" stage for free kicks, so this is when we choose kicker
        kicker_picker_.join_group([this](KickerPickerClient::Result result) {
            if (result.am_i_member && result.kicker_id == robot_id_) {
                set_current_position<FreeKicker>();
            } else {
                set_default_position();

                if (dynamic_cast<Offense*>(current_position_.get()) != nullptr) {
                    set_current_position<SmartIdle>();
                }
            }
        });

    } else if (current_play_state_.is_their_restart() && current_play_state_.is_free_kick()) {
        if (dynamic_cast<Offense*>(current_position_.get()) != nullptr ||
            dynamic_cast<PenaltyPlayer*>(current_position_.get()) != nullptr ||
            dynamic_cast<FreeKicker*>(current_position_.get()) != nullptr) {
            set_current_position<SmartIdle>();
        }
    }
}

void RobotFactoryPosition::update_position() {
    bool manual_position_set = set_position_override_if_requested();
    if (manual_position_set) {
        return;
    }

    // Robot 0, 1, and 2 maintain their assigned positions - never change
    // Robot 0: Idle, Robot 1: Dribbler, Robot 2: Idle
    if (robot_id_ == 0 || robot_id_ == 1 || robot_id_ == 2) {
        return;
    }

    switch (current_play_state_.state()) {
        case PlayState::State::Playing: {
            // We just became regular playing.
            set_default_position();
            break;
        }

        case PlayState::State::Setup:
        case PlayState::State::Ready: {
            // Currently in setup
            // Robot 0, 1, and 2 maintain their positions even during restarts
            if (robot_id_ == 0 || robot_id_ == 1 || robot_id_ == 2) {
                break;
            }

            // This is the only case where we have to do something on every tick
            if (current_play_state_.is_their_restart()) {  // Their restart
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
                    if (dynamic_cast<Offense*>(current_position_.get()) != nullptr) {
                        set_current_position<SmartIdle>();
                    }
                }
            }

            break;
        }

        case PlayState::State::PenaltyPlaying:
        case PlayState::State::Stop:
        case PlayState::State::Halt: {
            // No action needed on each tick
            break;
        }
    }
}

void RobotFactoryPosition::set_default_position() {
    // Get sorted positions of all friendly robots
    using RobotPos = std::pair<int, double>;  // (robotId, yPosition)

    // Robot 0, 1, and 2 maintain their assigned positions - never change
    if (robot_id_ == 0 || robot_id_ == 1 || robot_id_ == 2) {
        return;
    }
    
    // All other robots (except goalie) use SmartIdle for testing
    if (robot_id_ != goalie_id_) {
        set_current_position<SmartIdle>();
        return;
    }

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

    // This code is now unreachable for non-goalie robots due to the SmartIdle
    // assignment above, but keeping it for goalie and future use
    // Assigning new position
    // Checking whether we have possesion or if the ball is on their half
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

    // Robot 0 and 2 are idle and don't send communication requests
    // Skip adding communication requests for robots 0 and 2
    if (robot_id_ != 0 && robot_id_ != 2) {
        // Combine the two
        result.insert(result.end(), current.begin(), current.end());
    }
    // For robots 0 and 2, just return the base class requests (which should be empty)

    return result;
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

std::string RobotFactoryPosition::get_current_state() {
    return current_position_->get_current_state();
}

void RobotFactoryPosition::set_override_position(
    const strategy::OverridingPositions& overriding_position) {
    override_play_position_ = overriding_position;
}

/**
 * Checks override_play_position_, which automatically updates when an override is set.
 * If it is anything but auto, set the current position to that position and return true.
 */
bool RobotFactoryPosition::set_position_override_if_requested() {
    switch (override_play_position_) {
        case strategy::OverridingPositions::OFFENSE: {
            set_current_position<Offense>();
            return true;
        }
        case strategy::OverridingPositions::DEFENSE: {
            set_current_position<Defense>();
            return true;
        }
        case strategy::OverridingPositions::FREE_KICKER: {
            set_current_position<FreeKicker>();
            return true;
        }
        case strategy::OverridingPositions::PENALTY_PLAYER: {
            set_current_position<PenaltyPlayer>();
            return true;
        }
        case strategy::OverridingPositions::PENALTY_NON_KICKER: {
            set_current_position<PenaltyNonKicker>();
            return true;
        }
        case strategy::OverridingPositions::SMART_IDLE: {
            set_current_position<SmartIdle>();
            return true;
        }
        case strategy::OverridingPositions::SOLO_OFFENSE: {
            set_current_position<SoloOffense>();
            return true;
        }
        case strategy::OverridingPositions::ZONER: {
            set_current_position<Zoner>();
            return true;
        }
        case strategy::OverridingPositions::IDLE: {
            set_current_position<Idle>();
            return true;
        }
        default: {
            return false;
        }
    }
}

}  // namespace strategy