#include "offense.hpp"

namespace strategy {

Offense::Offense(int r_id) : Position(r_id) {
    position_name_ = "Offense";
    send_scorer_request();
    current_state_ = FACING;
    // if (r_id == 2) {
    //     current_state_ = STEALING;
    // } else {
    //     current_state_ = FACING;
    // }
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
                // SPDLOG_INFO("\033[92mRobot {} is finished passing\033[0m", robot_id_);
                SPDLOG_INFO("\033[92mRobot {} finished pass - is done\033[0m", robot_id_);
                next_state = IDLING;
            }

            if (distance_to_ball > ball_lost_distance_) {
                //     SPDLOG_INFO("\033[92mRobot {} is finished pass - ball_lost_distance\033[0m",
                //                 robot_id_);
                SPDLOG_INFO("\033[92mRobot {} finished pass\033[0m", robot_id_);
                next_state = IDLING;
            }
            break;
        case SHOOTING:
            // transition to idling if we no longer have the ball (i.e. it was passed or it was
            // stolen)
            if (distance_to_ball > ball_lost_distance_) {
                next_state = IDLING;
            }
            break;
        case RECEIVING:
            // transition to idling if we are close enough to the ball
            if (distance_to_ball < ball_receive_distance_) {
                next_state = IDLING;
            }
            break;
        case STEALING:
            // transition to idling if we are close enough to the ball
            if (check_is_done()) {
                /* SPDLOG_INFO("\033[92m ball pos {}{} \033[0m", ball_position.x(),
                 * ball_position.y()); */

                // send direct pass request to robot 4
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
        // Do nothing
        auto empty_motion_cmd = planning::MotionCommand{};
        intent.motion_command = empty_motion_cmd;
        return intent;
    } else if (current_state_ == SEARCHING) {
        // DEFINE SEARCHING BEHAVIOR
    } else if (current_state_ == PASSING) {
        // attempt to pass the ball to the target robot
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

void Offense::receive_communication_response(communication::AgentPosResponseWrapper response) {
    Position::receive_communication_response(response);


    // Check to see if we are dealing with scorer requests
    if (const communication::ScorerRequest* scorer_response = std::get_if<communication::ScorerRequest>(&response.associated_request)) {
        handle_scorer_response(response.responses);
        return;
    }
}

communication::PosAgentResponseWrapper Offense::receive_communication_request(communication::AgentPosRequestWrapper request) {
    communication::PosAgentResponseWrapper comm_response = Position::receive_communication_request(request);

    // If a scorer request was received override the position receive_communication_request return
    if (const communication::ScorerRequest* scorer_request = std::get_if<communication::ScorerRequest>(&request.request)) {
        communication::ScorerResponse scorer_response = receive_scorer_request(*scorer_request);
        comm_response.response = scorer_response;
    }

    return comm_response;
}

void Offense::send_scorer_request() {
    communication::ScorerRequest scorer_request{};
    communication::generate_uid(scorer_request);

    communication::PosAgentRequestWrapper communication_request{};
    communication_request.request = scorer_request;
    communication_request.broadcast = true;

    communication_request_ = communication_request;
}

communication::ScorerResponse Offense::receive_scorer_request(communication::ScorerRequest scorer_request) {
    communication::ScorerResponse scorer_response{};
    communication::generate_uid(scorer_response);
    scorer_response.robot_id = robot_id_;

    // Calculate distance to ball
    rj_geometry::Point robot_position = world_state()->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = world_state()->ball.position;
    double ball_distance = robot_position.dist_to(ball_position);
    scorer_response.ball_distance = ball_distance;

    return scorer_response;
}

void Offense::handle_scorer_response(std::vector<communication::AgentResponseVariant> responses) {
    rj_geometry::Point this_robot_position = world_state()->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = world_state()->ball.position;
    double this_ball_distance = this_robot_position.dist_to(ball_position);

    for (communication::AgentResponseVariant response : responses) {
        if (const communication::ScorerResponse* scorer_response = std::get_if<communication::ScorerResponse>(&response)) {
            if (scorer_response->ball_distance < this_ball_distance) {
                return;
            }
        }
    }

    // Make this robot the scorer
    scorer = true;
}

void Offense::derived_acknowledge_pass() { current_state_ = FACING; }

void Offense::derived_pass_ball() { current_state_ = PASSING; }

void Offense::derived_acknowledge_ball_in_transit() {
    current_state_ = RECEIVING;
    chasing_ball = false;
}

}  // namespace strategy
