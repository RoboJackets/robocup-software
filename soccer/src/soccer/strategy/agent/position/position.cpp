#include "position.hpp"

namespace strategy {

Position::Position(int r_id) : robot_id_(r_id) {}

std::optional<RobotIntent> Position::get_task() {
    // init an intent with our robot id
    RobotIntent intent = RobotIntent{};
    intent.robot_id = robot_id_;

    // if world_state invalid, return empty_intent
    if (!assert_world_state_valid()) {
        intent.motion_command = planning::EmptyMotionCommand{};
        return intent;
    }

    // delegate to derived class to complete behavior
    return derived_get_task(intent);
}

void Position::set_time_left(double time_left) { time_left_ = time_left; }

void Position::set_is_done() { is_done_ = true; }

void Position::set_goal_canceled() { goal_canceled_ = true; }

bool Position::check_is_done() {
    if (is_done_) {
        is_done_ = false;
        return true;
    }
    return false;
}

bool Position::check_goal_canceled() {
    if (goal_canceled_) {
        goal_canceled_ = false;
        return true;
    }
    return false;
}

void Position::update_world_state(WorldState world_state) {
    // mutex lock here as the world state could be accessed while callback from
    // AC is updating it = undefined behavior = crashes
    auto lock = std::lock_guard(world_state_mutex_);
    last_world_state_ = std::move(world_state);
}

void Position::update_coach_state(rj_msgs::msg::CoachState msg) {
    match_situation_ = msg.match_situation;
    our_possession_ = msg.our_possession;
    // TODO: how is planner supposed to get this global override info?
    global_override_ = msg.global_override;
    /* SPDLOG_INFO("match_situation {}, our_possession {}", match_situation_, our_possession_); */
}

[[nodiscard]] WorldState* Position::world_state() {
    // thread-safe getter for world_state (see update_world_state())
    auto lock = std::lock_guard(world_state_mutex_);
    return &last_world_state_;
}

bool Position::assert_world_state_valid() {
    WorldState* world_state = this->world_state();  // thread-safe getter
    if (world_state == nullptr) {
        SPDLOG_WARN("WorldState!");
        return false;
    }
    return true;
}

communication::PosAgentRequestWrapper Position::send_communication_request() {
    return communication_request_;
}

void Position::receive_communication_response([
    [maybe_unused]] 
communication::AgentPosResponseWrapper response) {}

communication::PosAgentResponseWrapper Position::receive_communication_request([
    [maybe_unused]] communication::AgentPosRequestWrapper request) {
    communication::PosAgentResponseWrapper pos_agent_response{};
    communication::Acknowledge response{};
    communication::generate_uid(response);
    pos_agent_response.response = response;
    return pos_agent_response;
}

rj_msgs::msg::RobotIntent Position::get_empty_intent() const {
    rj_msgs::msg::RobotIntent intent{};
    auto empty = rj_msgs::msg::EmptyMotionCommand{};
    intent.motion_command.empty_command = {empty};
    return intent;
}

const std::string Position::get_name() { return position_name_; }

}  // namespace strategy
