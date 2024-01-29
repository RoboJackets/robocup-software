#include "offense.hpp"

namespace strategy {

Offense::Offense(int r_id) : Position(r_id) {
    position_name_ = "Offense";
    current_state_ = IDLING;
}

std::optional<RobotIntent> Offense::derived_get_task(RobotIntent intent) {
    // SPDLOG_INFO("MY ID: {} in offense derived task!\n", robot_id_);
    current_state_ = update_state();
    // SPDLOG_INFO("My current offense state is {}", current_state_);
    return state_to_task(intent);
}

Offense::State Offense::update_state() {
    State next_state = current_state_;
    // handle transitions between current state
    WorldState* world_state = this->last_world_state_;

    // if no ball found, stop and return to box immediately
    if (!world_state->ball.visible) {
        return current_state_;
    }

    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = world_state->ball.position;
    double distance_to_ball = robot_position.dist_to(ball_position);

    if (current_state_ == IDLING) {
        send_scorer_request();
        next_state = SEARCHING;
    } else if (current_state_ == SEARCHING) {
        if (scorer_) {
            next_state = STEALING;
        }
    } else if (current_state_ == PASSING) {
        // transition to idling if we no longer have the ball (i.e. it was passed or it was
        // stolen)
        if (check_is_done()) {
            next_state = IDLING;
        }

        if (distance_to_ball > ball_lost_distance_) {
            next_state = IDLING;
        }
    } else if (current_state_ == PREPARING_SHOT) {
        if (check_is_done()) {
            next_state = SHOOTING;
        }
    } else if (current_state_ == SHOOTING) {
        // transition to idling if we no longer have the ball (i.e. it was passed or it was
        // stolen)
        if (check_is_done() || distance_to_ball > ball_lost_distance_) {
            send_reset_scorer_request();
            next_state = SEARCHING;
        }
    } else if (current_state_ == RECEIVING) {
        // transition to idling if we are close enough to the ball
        if (distance_to_ball < ball_receive_distance_) {
            next_state = IDLING;
        }
    } else if (current_state_ == STEALING) {
        // The collect planner check_is_done() is wonky so I added a second clause to check
        // distance
        if (check_is_done() || distance_to_ball < ball_receive_distance_) {
            // send direct pass request to robot 4
            if (scorer_) {
                next_state = PREPARING_SHOT;
            } else {
                /* send_direct_pass_request({4}); */
                /* next_state = SEARCHING; */
            }
        }
    } else if (current_state_ == FACING) {
        if (check_is_done()) {
            next_state = IDLING;
        }
    }

    return next_state;
}

std::optional<RobotIntent> Offense::state_to_task(RobotIntent intent) {
    float dist{0.0f};
    if (current_state_ == IDLING) {
        // Do nothing
        auto empty_motion_cmd = planning::MotionCommand{};
        intent.motion_command = empty_motion_cmd;
        return intent;
    } else if (current_state_ == SEARCHING) {
        // DEFINE SEARCHING BEHAVIOR
        auto empty_motion_cmd = planning::MotionCommand{};
        intent.motion_command = empty_motion_cmd;
        return intent;
    } else if (current_state_ == PASSING) {
        // attempt to pass the ball to the target robot
        rj_geometry::Point target_robot_pos =
            last_world_state_->get_robot(true, target_robot_id).pose.position();
        rj_geometry::Point this_robot_pos =
            last_world_state_->get_robot(true, this->robot_id_).pose.position();
        planning::LinearMotionInstant target{target_robot_pos};
        auto line_kick_cmd = planning::MotionCommand{"line_kick", target};
        intent.motion_command = line_kick_cmd;
        intent.shoot_mode = RobotIntent::ShootMode::KICK;
        // NOTE: Check we can actually use break beams
        intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
        // Adjusts kick speed based on distance. Refer to
        // TIGERS Mannheim eTDP from 2019 for details
        // See also passer.py in rj_gameplay
        dist = target_robot_pos.dist_to(this_robot_pos);
        intent.kick_speed = std::sqrt((std::pow(kFinalBallSpeed, 2)) - (2 * kBallDecel * dist));
        intent.is_active = true;
        return intent;
    } else if (current_state_ == PREPARING_SHOT) {
        // pivot around ball...
        auto ball_pt = last_world_state_->ball.position;

        // ...to face their goal
        rj_geometry::Point their_goal_pos = field_dimensions_.their_goal_loc();
        planning::LinearMotionInstant target_instant{their_goal_pos};

        auto pivot_cmd = planning::MotionCommand{"pivot"};
        pivot_cmd.target = target_instant;
        pivot_cmd.pivot_point = ball_pt;
        intent.motion_command = pivot_cmd;
        intent.dribbler_speed = 255.0;
        return intent;
    } else if (current_state_ == SHOOTING) {
        rj_geometry::Point their_goal_pos = field_dimensions_.their_goal_loc();
        planning::LinearMotionInstant target{their_goal_pos};
        auto line_kick_cmd = planning::MotionCommand{"line_kick", target};
        intent.motion_command = line_kick_cmd;
        intent.shoot_mode = RobotIntent::ShootMode::KICK;
        intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
        intent.kick_speed = 4.0;
        intent.is_active = true;
        return intent;
    } else if (current_state_ == RECEIVING) {
        // check how far we are from the ball
        rj_geometry::Point robot_position =
            last_world_state_->get_robot(true, robot_id_).pose.position();
        rj_geometry::Point ball_position = last_world_state_->ball.position;
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
        // if ball fast, use settle, otherwise collect
        if (last_world_state_->ball.velocity.mag() > 0.75) {
            auto settle_cmd = planning::MotionCommand{"settle"};
            intent.motion_command = settle_cmd;
            intent.dribbler_speed = 255.0;
            return intent;
        } else {
            auto collect_cmd = planning::MotionCommand{"collect"};
            intent.motion_command = collect_cmd;
            intent.dribbler_speed = 255.0;
            return intent;
        }
    } else if (current_state_ == FACING) {
        rj_geometry::Point robot_position =
            last_world_state_->get_robot(true, robot_id_).pose.position();
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
    if (const communication::ScorerRequest* scorer_response =
            std::get_if<communication::ScorerRequest>(&response.associated_request)) {
        handle_scorer_response(response.responses);
        return;
    }
}

communication::PosAgentResponseWrapper Offense::receive_communication_request(
    communication::AgentPosRequestWrapper request) {
    communication::PosAgentResponseWrapper comm_response =
        Position::receive_communication_request(request);

    // If a scorer request was received override the position receive_communication_request return
    if (const communication::ScorerRequest* scorer_request =
            std::get_if<communication::ScorerRequest>(&request.request)) {
        communication::ScorerResponse scorer_response = receive_scorer_request(*scorer_request);
        comm_response.response = scorer_response;
    } else if (const communication::ResetScorerRequest* _ =
                   std::get_if<communication::ResetScorerRequest>(&request.request)) {
        communication::Acknowledge response = receive_reset_scorer_request();
        comm_response.response = response;
    }

    return comm_response;
}

void Offense::send_scorer_request() {
    communication::ScorerRequest scorer_request{};
    communication::generate_uid(scorer_request);
    scorer_request.robot_id = robot_id_;

    // Calculate distance to ball
    rj_geometry::Point robot_position =
        last_world_state_->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = last_world_state_->ball.position;
    double ball_distance = robot_position.dist_to(ball_position);
    scorer_request.ball_distance = ball_distance;

    communication::PosAgentRequestWrapper communication_request{};
    communication_request.request = scorer_request;
    communication_request.broadcast = true;

    communication_request_ = communication_request;
}

void Offense::send_reset_scorer_request() {
    communication::ResetScorerRequest reset_scorer_request{};
    communication::generate_uid(reset_scorer_request);

    communication::PosAgentRequestWrapper communication_request{};
    communication_request.request = reset_scorer_request;
    communication_request.broadcast = true;

    communication_request_ = communication_request;
    last_scorer_ = true;
    scorer_ = false;
}

communication::ScorerResponse Offense::receive_scorer_request(
    communication::ScorerRequest scorer_request) {
    communication::ScorerResponse scorer_response{};
    communication::generate_uid(scorer_response);
    scorer_response.robot_id = robot_id_;

    // Calculate distance to ball
    rj_geometry::Point robot_position =
        last_world_state_->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = last_world_state_->ball.position;
    double ball_distance = robot_position.dist_to(ball_position);
    scorer_response.ball_distance = ball_distance;

    // Switch scorers if better scorer
    if (scorer_ && scorer_request.ball_distance < ball_distance) {
        scorer_ = false;
        current_state_ = FACING;
    }

    // Give fake answer if previous scorer
    if (last_scorer_) {
        scorer_response.ball_distance = 300;
    }

    return scorer_response;
}

communication::Acknowledge Offense::receive_reset_scorer_request() {
    communication::Acknowledge acknowledge{};
    communication::generate_uid(acknowledge);

    last_scorer_ = false;
    send_scorer_request();

    return acknowledge;
}

void Offense::handle_scorer_response(
    const std::vector<communication::AgentResponseVariant>& responses) {
    rj_geometry::Point this_robot_position =
        last_world_state_->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = last_world_state_->ball.position;
    double this_ball_distance = this_robot_position.dist_to(ball_position);

    for (communication::AgentResponseVariant response : responses) {
        if (const communication::ScorerResponse* scorer_response =
                std::get_if<communication::ScorerResponse>(&response)) {
            if (scorer_response->ball_distance < this_ball_distance) {
                return;
            }
        }
    }

    // Make this robot the scorer
    scorer_ = true;
}

void Offense::derived_acknowledge_pass() { current_state_ = FACING; }

void Offense::derived_pass_ball() { current_state_ = PASSING; }

void Offense::derived_acknowledge_ball_in_transit() {
    current_state_ = RECEIVING;
    chasing_ball = false;
}

}  // namespace strategy
