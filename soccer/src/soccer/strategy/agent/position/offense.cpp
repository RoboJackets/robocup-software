#include "offense.hpp"

namespace strategy {

Offense::Offense(int r_id) : Position(r_id) { 
    SPDLOG_INFO("\033[92m Robot Id: {}\033[0m", r_id);
    position_name_ = "Offense";
    if (r_id == 2) {
        current_state_ = STEALING;
        SPDLOG_INFO("\033[92mInitializing Stealing Robot\033[0m");
    } else {
        current_state_ = FACING;
    current_state_ = update_state();
    if (robot_id_ == 2) {
        SPDLOG_INFO("\033[92m current_state_ {} \033[0m", current_state_);
    }
    return state_to_task(intent);
}

Offense::State Offense::update_state() {
    State next_state = current_state_;
    // handle transitions between current state
    WorldState* world_state = this->world_state();
    
    // if no ball found, stop and return to box immediately
    if (!world_state->ball.visible) {
        SPDLOG_INFO("\033[92m BALL INVIS!! \033[0m");
        return current_state_;
    }

    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = world_state->ball.position;
    double distance_to_ball = robot_position.dist_to(ball_position);

    switch (current_state_) {
        case IDLING:
            break;
        case SEARCHING:
            break;
        case PASSING:
            // transition to idling if we no longer have the ball (i.e. it was passed or it was 
            // stolen)
            if (distance_to_ball > BALL_LOST_DISTANCE) {
                next_state = IDLING;
            }
            break;
        case SHOOTING:
            // transition to idling if we no longer have the ball (i.e. it was passed or it was 
            // stolen)
            if (distance_to_ball > BALL_LOST_DISTANCE) {
                next_state = IDLING;
            }
            break;
        case RECEIVING:
            // transition to idling if we are close enough to the ball
            if (distance_to_ball < BALL_RECEIVE_DISTANCE) {
                SPDLOG_INFO("\033[92mRobot {} is {} from the ball\033[0m", robot_id_, distance_to_ball);
                next_state = IDLING;
            }
            break;
        case STEALING:
            // transition to idling if we are close enough to the ball
            if (check_is_done()) {
                /* SPDLOG_INFO("\033[92m ball pos {}{} \033[0m", ball_position.x(), ball_position.y()); */
                SPDLOG_INFO("\033[92m switching off stealing {} \033[0m", BALL_RECEIVE_DISTANCE);

                // send direct pass request to robot 3
                send_direct_pass_request({1});
                
                // go to IDLING (pass receieved will go to PASSING)
                next_state = SEARCHING;
            }
            break;
    }

    return next_state;
}

std::optional<RobotIntent> Offense::state_to_task(RobotIntent intent) {
    if (current_state_ == IDLING) {
        // DO NOTHING
        /* SPDLOG_INFO("\033[93m Robot {} is idling\033[0m", robot_id_); */
    } else if (current_state_ == SEARCHING) {
        // TODO: DEFINE SEARCHING BEHAVIOR
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
    } else if (current_state_ == SHOOTING) {
        // TODO: Shoot the ball at the goal
    } else if (current_state_ == RECEIVING) {
        SPDLOG_INFO("\033[94mRobot {} is receiving the ball\033[0m", robot_id_);
        // intercept the bal
        rj_geometry::Point ball_position = world_state()->ball.position;
        auto intercept_cmd = planning::InterceptMotionCommand{ball_position};
        intent.motion_command = intercept_cmd;
        intent.motion_command_name = fmt::format("robot {} offensive receive ball", robot_id_);
        return intent;
    } else if (current_state_ == STEALING) {
        /* SPDLOG_INFO("\033[93mRobot {} is stealing the ball\033[0m", robot_id_); */
        // intercept the ball
        auto collect_cmd = planning::CollectMotionCommand{};
        // rj_geometry::Point ball_position = world_state()->ball.position;
        // auto intercept_cmd = planning::InterceptMotionCommand{ball_position};
        // intent.motion_command = intercept_cmd;
        intent.motion_command = collect_cmd;
        intent.motion_command_name = fmt::format("robot {} offensive stealing ball", robot_id_);
        return intent;
    }

    // should be impossible to reach, but this is an EmptyMotionCommand
    return std::nullopt;
}

void Offense::receive_communication_response(communication::AgentPosResponseWrapper response) {
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
        //     SPDLOG_INFO("\033[91mRobot {} has sent a test response with message: {}\033[0m",
        //                 response.received_robot_ids[i], test_response->message);
        // }
    }

    // should be impossible to reach, but this is an EmptyMotionCommand
    return std::nullopt;
}

communication::PosAgentResponseWrapper Offense::receive_communication_request(
    communication::AgentPosRequestWrapper request) {
    communication::PosAgentResponseWrapper comm_response{};
    if (const communication::PassRequest* pass_request =
            std::get_if<communication::PassRequest>(&request.request)) {
        communication::PassResponse pass_response = receive_pass_request(*pass_request);
        comm_response.response = pass_response;
    } else if (const communication::IncomingPassRequest* incoming_pass_request = std::get_if<communication::IncomingPassRequest>(&request.request)) {
        communication::Acknowledge incoming_pass_acknowledge = acknowledge_pass(*incoming_pass_request);
        comm_response.response = incoming_pass_acknowledge;
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

communication::Acknowledge Offense::acknowledge_pass(communication::IncomingPassRequest incoming_pass_request) {
    communication::Acknowledge acknowledge_response{};
    communication::generate_uid(acknowledge_response);

    current_state_ = RECEIVING;

    return acknowledge_response;
}

void Offense::pass_ball(int robot_id) {
    target_robot_id = robot_id;
    current_state_ = PASSING;
}

}  // namespace strategy
