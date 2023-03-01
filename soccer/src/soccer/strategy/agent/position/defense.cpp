#include "defense.hpp"

namespace strategy {

Defense::Defense(int r_id) : Position(r_id) { position_name_ = "Defense"; }

std::optional<RobotIntent> Defense::derived_get_task(RobotIntent intent) {
    current_state_ = update_state();
    return state_to_task(intent);
}

Defense::State Defense::update_state() {
    State next_state = current_state_;
    // handle transitions between states
    WorldState* world_state = this->world_state();

    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = world_state->ball.position;
    double distance_to_ball = robot_position.dist_to(ball_position);

    switch (current_state_) {
        case IDLING:
            break;
        case SEARCHING:
            break;
        case RECEIVING:
            // transition to idling if we are close enough to the ball
            if (distance_to_ball < BALL_RECEIVE_DISTANCE) {
                next_state = IDLING;
            }
            break;
        case PASSING:
            // transition to idling if we no longer have the ball (i.e. it was passed or it was 
            // stolen)
            if (distance_to_ball > BALL_LOST_DISTANCE) {
                next_state = IDLING;
            }
            break;
    }

    return next_state;
}

std::optional<RobotIntent> Defense::state_to_task(RobotIntent intent) {
    if (current_state_ == IDLING) {
        // DO NOTHING
    } else if (current_state_ == SEARCHING) {
        // TODO: Define defensive searching behavior
    } else if (current_state_ == RECEIVING) {
        // intercept the bal
        rj_geometry::Point current_position = world_state()->get_robot(true, robot_id_).pose.position();
        auto intercept_cmd = planning::InterceptMotionCommand{current_position};
        intent.motion_command = intercept_cmd;
        intent.motion_command_name = fmt::format("robot {} defensive receive ball", robot_id_);
        return intent;
    } else if (current_state_ == PASSING) {
        // attempt to pass the ball to the target robot
        rj_geometry::Point target_robot_pos = world_state()->get_robot(true, target_robot_id).pose.position();
        auto line_kick_cmd = planning::LineKickMotionCommand{target_robot_pos};
        intent.motion_command = line_kick_cmd;
        intent.motion_command_name = fmt::format("robot {} offensive pass to robot {}", robot_id_, target_robot_id);
        intent.shoot_mode = RobotIntent::ShootMode::KICK;
        // NOTE: Check we can actually use break beams
        intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
        // TODO: Adjust the kick speed based on distance
        intent.kick_speed = 4.0;
        intent.is_active = true;
        return intent;
    }

    return std::nullopt;
}

void Defense::receive_communication_response(communication::AgentPosResponseWrapper response) {
    for (u_int32_t i = 0; i < response.responses.size(); i++) {
        if (const communication::Acknowledge* acknowledge =
                std::get_if<communication::Acknowledge>(&response.responses[i])) {
            // if the acknowledgement is from an incoming pass request -> pass the ball
            if (const communication::IncomingPassRequest* incoming_pass_request = std::get_if<communication::IncomingPassRequest>(&response.associated_request)) {
                pass_ball(response.received_robot_ids[i]);
            }

        } else if (const communication::PassResponse* pass_response =
                       std::get_if<communication::PassResponse>(&response.responses[i])) {
            // get the associated pass request for this response
            if (const communication::PassRequest* sent_pass_request = std::get_if<communication::PassRequest>(&response.associated_request)) {
                if (sent_pass_request->direct) {
                    // if direct -> pass to first robot
                    send_pass_confirmation(response.received_robot_ids[i]);
                } else {
                    // TODO: handle deciding on indirect passing
                }
            }
        }

        // TEST CODE: UNCOMMENT TO TEST
        // if (const communication::TestResponse* test_response =
        //                std::get_if<communication::TestResponse>(&response.responses[i])) {
        //     SPDLOG_INFO("\033[92m Robot {} sent the test response {}\033[0m",
        //                 response.received_robot_ids[i], test_response->message);
        // }
    }
}

communication::PosAgentResponseWrapper Offense::receive_communication_request(
    communication::AgentPosRequestWrapper request) {
    communication::PosAgentResponseWrapper comm_response{};
    SPDLOG_INFO("\033[92m offense recv comm req !! \033[0m");
    if (const communication::PassRequest* pass_request =
            std::get_if<communication::PassRequest>(&request.request)) {
        communication::PassResponse pass_response = receive_pass_request(*pass_request);
        comm_response.response = pass_response;
        // TODO: "IncomingPassRequest" => "IncomingBallRequest" (or smth)
    } else if (const communication::IncomingPassRequest* incoming_pass_request = std::get_if<communication::IncomingPassRequest>(&request.request)) {
        communication::Acknowledge incoming_pass_acknowledge = acknowledge_pass(*incoming_pass_request);
        comm_response.response = incoming_pass_acknowledge;
    } else if (const communication::BallInTransitRequest* ball_in_transit_request = std::get_if<communication::BallInTransitRequest>(&request.request)) {
        communication::Acknowledge ball_in_transit_acknowledge = acknowledge_ball_in_transit(*ball_in_transit_request);
        comm_response.response = ball_in_transit_acknowledge;
    } else {
        communication::Acknowledge acknowledge{};
        communication::generate_uid(acknowledge);
        comm_response.response = acknowledge;
    }

    // TEST CODE: UNCOMMENT TO TEST
    // if (const communication::TestRequest* test_request =
    //                std::get_if<communication::TestRequest>(&request.request)) {
    //     communication::TestResponse test_response{};
    //     test_response.message = fmt::format("An offensive player (robot: {}) says hi", robot_id_);
    //     communication::generate_uid(test_response);
    //     comm_response.response = test_response;
    // }

    return comm_response;
}

void Offense::pass_ball(int robot_id) {
    target_robot_id = robot_id;
    current_state_ = PASSING;

    communication::BallInTransitRequest ball_in_transit_request{};
    communication::generate_uid(ball_in_transit_request);
    
    communication::PosAgentRequestWrapper communication_request{};
    communication_request.request = ball_in_transit_request;
    communication_request.target_agents = {robot_id};
    communication_request.urgent = true;
    communication_request.broadcast = false;

    communication_request_ = communication_request;
}

communication::Acknowledge Offense::acknowledge_ball_in_transit(communication::BallInTransitRequest ball_in_transit_request) {
    communication::Acknowledge acknowledge_response{};
    communication::generate_uid(acknowledge_response);

    current_state_ = RECEIVING;

    return acknowledge_response;
}

}  // namespace strategy
