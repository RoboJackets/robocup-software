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
        // check how far we are from the ball
        rj_geometry::Point robot_position =
            world_state()->get_robot(true, robot_id_).pose.position();
        rj_geometry::Point ball_position = world_state()->ball.position;
        double distance_to_ball = robot_position.dist_to(ball_position);
        if (distance_to_ball > max_receive_distance && !chasing_ball) {
            auto motion_instance =
                planning::LinearMotionInstant{robot_position, rj_geometry::Point{0.0, 0.0}};
            auto face_ball = planning::FaceBall{};
            auto face_ball_cmd = planning::MotionCommand{"path_target", motion_instance, face_ball};
            intent.motion_command = face_ball_cmd;
        } else {
            // intercept the bal
            chasing_ball = true;
            SPDLOG_INFO("\033[92mrobot {} settling the ball\033[0m", robot_id_);
            auto collect_cmd = planning::MotionCommand{"collect"};
            intent.motion_command = collect_cmd;
        }
        return intent;
    } else if (current_state_ == PASSING) {
        // attempt to pass the ball to the target robot
        SPDLOG_INFO("\033[92mrobot {} passing ball\033[0m", robot_id_);
        rj_geometry::Point target_robot_pos =
            world_state()->get_robot(true, target_robot_id).pose.position();
        planning::LinearMotionInstant target{target_robot_pos};
        auto line_kick_cmd = planning::MotionCommand{"line_kick", target};
        intent.motion_command = line_kick_cmd;
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

communication::PosAgentResponseWrapper Defense::receive_communication_request(
    communication::AgentPosRequestWrapper request) {
    communication::PosAgentResponseWrapper comm_response;
    if (const communication::PassRequest* pass_request =
            std::get_if<communication::PassRequest>(&request.request)) {
        communication::PassResponse pass_response = receive_pass_request(pass_request);
        comm_response.response = pass_response;
    } else if (const communication::IncomingPassRequest* incoming_pass_request = std::get_if<communication::IncomingPassRequest>(&request.request)) {
        communication::Acknowledge incoming_pass_acknowledge = confirm_pass(incoming_pass_request);
        // TODO: Set FSM state to receiving pass
        comm_response.response = incoming_pass_acknowledge;
    } else if (const communication::PositionRequest* position_request =
                   std::get_if<communication::PositionRequest>(&request.request)) {
        communication::PositionResponse position_response{};
        position_response.position = position_name_;
        communication::generate_uid(position_response);
        comm_response.response = position_response;
    } else if (const communication::TestRequest* test_request =
                   std::get_if<communication::TestRequest>(&request.request)) {
        communication::TestResponse test_response{};
        test_response.message = fmt::format("robot {} says hello", robot_id_);
        communication::generate_uid(test_response);
        comm_response.response = test_response;
    } else {
        communication::Acknowledge acknowledge{};
        communication::generate_uid(acknowledge);
        comm_response.response = acknowledge;
    }

communication::Acknowledge Defense::acknowledge_ball_in_transit(
    communication::BallInTransitRequest ball_in_transit_request) {
    // Call to super
    communication::Acknowledge acknowledge_response =
        Position::acknowledge_ball_in_transit(ball_in_transit_request);
    // Update current state
    current_state_ = RECEIVING;
    // Return acknowledge response
    return acknowledge_response;
}

}  // namespace strategy
