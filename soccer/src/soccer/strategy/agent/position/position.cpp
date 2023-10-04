#include "position.hpp"

namespace strategy {

Position::Position(int r_id) : robot_id_(r_id) {}

std::optional<RobotIntent> Position::get_task() {
    // init an intent with our robot id
    RobotIntent intent = RobotIntent{};
    intent.robot_id = robot_id_;

    // if world_state invalid, return empty MotionCommand (equivalent to HALT)
    if (!assert_world_state_valid()) {
        intent.motion_command = planning::MotionCommand{};
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
    our_possession_ = msg.our_possession;
    // TODO: how is planner supposed to get this global override info?
    global_override_ = msg.global_override;
    /* SPDLOG_INFO("match_restart {}, match_state {}, our_possession {}", match_restart_,
     * match_state_, our_possession_); */
}

void Position::update_play_state(rj_msgs::msg::PlayState msg) {
    current_play_state_ = msg;
}

void Position::update_field_dimensions(FieldDimensions field_dims) {
    field_dimensions_ = std::move(field_dims);
}

void Position::update_alive_robots(std::vector<u_int8_t> alive_robots) {
    alive_robots_ = alive_robots;

    if (alive &&
        std::find(alive_robots_.begin(), alive_robots_.end(), robot_id_) != alive_robots_.end()) {
        alive = false;
        die();
    } else if (!alive && std::find(alive_robots_.begin(), alive_robots_.end(), robot_id_) ==
                             alive_robots_.end()) {
        alive = true;
        revive();
    }
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

std::optional<communication::PosAgentRequestWrapper> Position::send_communication_request() {
    if (communication_request_ != std::nullopt) {
        auto saved_comm_req = communication_request_;
        communication_request_ = std::nullopt;
        return saved_comm_req;
    }
    return std::nullopt;
}

void Position::receive_communication_response(communication::AgentPosResponseWrapper response) {
    for (u_int32_t i = 0; i < response.responses.size(); i++) {
        if (const communication::Acknowledge* acknowledge =
                std::get_if<communication::Acknowledge>(&response.responses[i])) {
            // if the acknowledgement is from an incoming pass request -> pass the ball
            if (const communication::IncomingBallRequest* incoming_ball_request =
                    std::get_if<communication::IncomingBallRequest>(&response.associated_request)) {
                pass_ball(response.received_robot_ids[i]);
            }

        } else if (const communication::PassResponse* pass_response =
                       std::get_if<communication::PassResponse>(&response.responses[i])) {
            // get the associated pass request for this response
            if (const communication::PassRequest* sent_pass_request =
                    std::get_if<communication::PassRequest>(&response.associated_request)) {
                if (sent_pass_request->direct) {
                    // if direct -> pass to first robot
                    send_pass_confirmation(response.received_robot_ids[i]);
                } else {
                    // TODO: handle deciding on indirect passing
                }
            }
        }
    }
}

communication::PosAgentResponseWrapper Position::receive_communication_request(
    communication::AgentPosRequestWrapper request) {
    communication::PosAgentResponseWrapper comm_response{};
    if (const communication::PassRequest* pass_request =
            std::get_if<communication::PassRequest>(&request.request)) {
        communication::PassResponse pass_response = receive_pass_request(*pass_request);
        comm_response.response = pass_response;
    } else if (const communication::IncomingBallRequest* incoming_ball_request =
                   std::get_if<communication::IncomingBallRequest>(&request.request)) {
        communication::Acknowledge incoming_pass_acknowledge =
            acknowledge_pass(*incoming_ball_request);
        comm_response.response = incoming_pass_acknowledge;
    } else if (const communication::BallInTransitRequest* ball_in_transit_request =
                   std::get_if<communication::BallInTransitRequest>(&request.request)) {
        communication::Acknowledge ball_in_transit_acknowledge =
            acknowledge_ball_in_transit(*ball_in_transit_request);
        comm_response.response = ball_in_transit_acknowledge;
    } else {
        communication::Acknowledge acknowledge{};
        communication::generate_uid(acknowledge);
        comm_response.response = acknowledge;
    }

    return comm_response;
}

const std::string Position::get_name() { return position_name_; }

void Position::send_direct_pass_request(std::vector<u_int8_t> target_robots) {
    communication::PassRequest pass_request{};
    communication::generate_uid(pass_request);
    pass_request.direct = true;
    pass_request.from_robot_id = robot_id_;

    communication::PosAgentRequestWrapper communication_request{};
    communication_request.request = pass_request;
    communication_request.target_agents = target_robots;
    communication_request.urgent = true;
    communication_request.broadcast = false;
    communication_request_ = communication_request;
}

communication::PassResponse Position::receive_pass_request(
    communication::PassRequest pass_request) {
    communication::PassResponse pass_response{};
    communication::generate_uid(pass_response);

    if (pass_request.direct) {
        // Handle direct pass request
        // TODO: Make this rely on actually being open
        pass_response.direct_open = true;
    } else {
        // TODO: Handle indirect pass request
        pass_response.direct_open = false;
    }

    return pass_response;
}

void Position::send_pass_confirmation(u_int8_t target_robot) {
    communication::IncomingBallRequest incoming_ball_request{};
    incoming_ball_request.from_robot_id = robot_id_;
    communication::generate_uid(incoming_ball_request);

    communication::PosAgentRequestWrapper communication_request{};
    communication_request.request = incoming_ball_request;
    communication_request.target_agents = {target_robot};
    communication_request.broadcast = false;
    communication_request.urgent = true;

    communication_request_ = communication_request;
}

communication::Acknowledge Position::acknowledge_pass(
    communication::IncomingBallRequest incoming_ball_request) {
    communication::Acknowledge acknowledge_response{};
    communication::generate_uid(acknowledge_response);

    face_robot_id = incoming_ball_request.from_robot_id;

    derived_acknowledge_pass();

    return acknowledge_response;
}

void Position::pass_ball(int robot_id) {
    target_robot_id = robot_id;

    communication::BallInTransitRequest ball_in_transit_request{};
    ball_in_transit_request.from_robot_id = robot_id_;
    communication::generate_uid(ball_in_transit_request);

    communication::PosAgentRequestWrapper communication_request{};
    communication_request.request = ball_in_transit_request;
    communication_request.target_agents = {(u_int8_t)robot_id};
    communication_request.urgent = true;
    communication_request.broadcast = false;

    communication_request_ = communication_request;

    derived_pass_ball();
}

communication::Acknowledge Position::acknowledge_ball_in_transit(
    communication::BallInTransitRequest ball_in_transit_request) {
    communication::Acknowledge acknowledge_response{};
    communication::generate_uid(acknowledge_response);

    face_robot_id = ball_in_transit_request.from_robot_id;

    derived_acknowledge_ball_in_transit();

    return acknowledge_response;
}

}  // namespace strategy
