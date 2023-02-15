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
    }
}

std::optional<RobotIntent> Offense::derived_get_task(RobotIntent intent) {
    current_state_ = update_state();
    return state_to_task(intent);
}

Offense::State Offense::update_state() {
    State next_state = current_state_;
    // handle transitions between current state
    WorldState* world_state = this->world_state();

    // if no ball found, stop and return to box immediately
    if (!world_state->ball.visible) {
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
            if (check_is_done()) {
                SPDLOG_INFO("\033[92mRobot {} is finished passing\033[0m", robot_id_);
                next_state = IDLING;
            }

            if (distance_to_ball > BALL_LOST_DISTANCE) {
                SPDLOG_INFO("\033[92mRobot {} is finished pass - ball_lost_distance\033[0m",
                            robot_id_);
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
                SPDLOG_INFO("\033[92mRobot {} is {} from the ball\033[0m", robot_id_,
                            distance_to_ball);
                next_state = IDLING;
            }
            break;
        case STEALING:
            // transition to idling if we are close enough to the ball
            if (check_is_done()) {
                /* SPDLOG_INFO("\033[92m ball pos {}{} \033[0m", ball_position.x(),
                 * ball_position.y()); */
                SPDLOG_INFO("\033[92m switching off stealing {} \033[0m", BALL_RECEIVE_DISTANCE);

                // send direct pass request to robot 3
                send_direct_pass_request({4});

                // go to IDLING (pass received will go to PASSING)
                next_state = SEARCHING;
            }
            break;
        case FACING:
            if (check_is_done()) {
                next_state = IDLING;
            }
    }

    return next_state;
}

std::optional<RobotIntent> Offense::state_to_task(RobotIntent intent) {
    if (current_state_ == IDLING) {
        // Face the ball
    } else if (current_state_ == SEARCHING) {
        // TODO: DEFINE SEARCHING BEHAVIOR
    } else if (current_state_ == PASSING) {
        // TODO: FIX LINE KICK
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
    } else if (current_state_ == SHOOTING) {
        // TODO: Shoot the ball at the goal
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
    } else if (current_state_ == STEALING) {
        // intercept the ball
        auto collect_cmd = planning::MotionCommand{"collect"};
        intent.motion_command = collect_cmd;
        return intent;
    } else if (current_state_ == FACING) {
        rj_geometry::Point robot_position =
            world_state()->get_robot(true, robot_id_).pose.position();
        auto current_location_instant =
            planning::LinearMotionInstant{robot_position, rj_geometry::Point{0.0, 0.0}};
        auto face_ball = planning::FaceBall{};
        auto face_ball_cmd =
            planning::MotionCommand{"path_target", current_location_instant, face_ball};
        intent.motion_command = face_ball_cmd;
        return intent;
    }

    // should be impossible to reach, but this is an EmptyMotionCommand
    return std::nullopt;
}

communication::PosAgentResponseWrapper Offense::receive_communication_request(
    communication::AgentPosRequestWrapper request) {
    communication::PosAgentResponseWrapper comm_response{};
    if (const communication::PassRequest* pass_request =
            std::get_if<communication::PassRequest>(&request.request)) {
        communication::PassResponse pass_response = receive_pass_request(pass_request);
        comm_response.response = pass_response;
    } else if (const communication::IncomingPassRequest* incoming_pass_request = std::get_if<communication::IncomingPassRequest>(&request.request)) {
        communication::Acknowledge incoming_pass_acknowledge = confirm_pass(incoming_pass_request);
        // TODO: Set FSM state for receiving pass
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
        test_response.message = fmt::format("An offensive player (robot: {}) says hi", robot_id_);
        communication::generate_uid(test_response);
        comm_response.response = test_response;
    } else {
        communication::Acknowledge acknowledge{};
        communication::generate_uid(acknowledge);
        comm_response.response = acknowledge;
    }
    return comm_response;
}

}  // namespace strategy
