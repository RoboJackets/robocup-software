#include "offense.hpp"

namespace strategy {

Offense::Offense(int r_id) : Position(r_id) {
    position_name_ = "Offense";
    current_state_ = STEALING;
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

    if (current_state_ == MARKING) {
        shoot_start_time_valid_ = false;
        SPDLOG_INFO("MARKING");
        if (RJ::now() > marking_timeout_) {
            next_state = STEALING;
        }
    } else if (current_state_ == STEALING) {
        SPDLOG_INFO("STEALING");
        // SKIP STEALING
        /* if (check_is_done() && distance_to_ball < ball_lost_distance_) { */
        next_state = PREPARING_SHOT;
        /* } */
    } else if (current_state_ == PREPARING_SHOT) {
        // start timeout for shooting
        if (!shoot_start_time_valid_) {
            shoot_start_time_valid_ = true;
            shoot_start_time_ = RJ::now();
            SPDLOG_INFO("validating shoot start time");
        }

        // if taking too long give up
        if (shoot_start_time_valid_ && (RJ::now() - shoot_start_time_ > max_shoot_duration_)) {
            SPDLOG_INFO("shoot_start_time_valid_ {}", shoot_start_time_valid_);
            SPDLOG_INFO("elapsed {}", (RJ::now() - shoot_start_time_).count());

            next_state = MARKING;
            marking_timeout_ = RJ::now() + RJ::Seconds(3);
            shoot_start_time_valid_ = false;
            SPDLOG_INFO("timeout shoot start time");
        }

        SPDLOG_INFO("PREPARING_SHOT");
        if (check_is_done()) {
            SPDLOG_INFO("done PREPARING_SHOT");
            next_state = SHOOTING;
        }
    } else if (current_state_ == SHOOTING) {
        // start timeout for shooting
        if (!shoot_start_time_valid_) {
            shoot_start_time_valid_ = true;
            shoot_start_time_ = RJ::now();
            SPDLOG_INFO("validating shoot start time");
        }

        // if taking too long give up
        if (shoot_start_time_valid_ && (RJ::now() - shoot_start_time_ > max_shoot_duration_)) {
            SPDLOG_INFO("shoot_start_time_valid_ {}", shoot_start_time_valid_);
            SPDLOG_INFO("elapsed {}", (RJ::now() - shoot_start_time_).count());

            next_state = MARKING;
            marking_timeout_ = RJ::now() + RJ::Seconds(3);
            shoot_start_time_valid_ = false;
            SPDLOG_INFO("timeout shoot start time");
        }

        SPDLOG_INFO("SHOOTING");
        if (check_is_done()) {
            next_state = MARKING;
            shoot_start_time_valid_ = false;
            marking_timeout_ = RJ::now() + RJ::Seconds(3);
        }
    }

    return next_state;
}

std::optional<RobotIntent> Offense::state_to_task(RobotIntent intent) {
    if (current_state_ == MARKING) {
        Marker marker{};
        return marker.get_task(intent, world_state(), this->field_dimensions_);
    } else if (current_state_ == STEALING) {
        // intercept the ball
        // if ball fast, use settle, otherwise collect
        if (world_state()->ball.velocity.mag() > 0.75) {
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
    } else if (current_state_ == PREPARING_SHOT) {
        // pivot around ball...
        auto ball_pt = world_state()->ball.position;

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
    rj_geometry::Point robot_position = world_state()->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = world_state()->ball.position;
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
}

communication::ScorerResponse Offense::receive_scorer_request(
    communication::ScorerRequest scorer_request) {
    communication::ScorerResponse scorer_response{};
    communication::generate_uid(scorer_response);
    scorer_response.robot_id = robot_id_;

    // Calculate distance to ball
    rj_geometry::Point robot_position = world_state()->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = world_state()->ball.position;
    double ball_distance = robot_position.dist_to(ball_position);
    scorer_response.ball_distance = ball_distance;

    // Switch scorers if better scorer
    if (scorer_ && scorer_request.ball_distance < ball_distance) {
        current_state_ = PREPARING_SHOT;
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
        world_state()->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = world_state()->ball.position;
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

void Offense::derived_acknowledge_pass() { current_state_ = PREPARING_SHOT; }

void Offense::derived_pass_ball() { current_state_ = PREPARING_SHOT; }

void Offense::derived_acknowledge_ball_in_transit() {
    current_state_ = PREPARING_SHOT;
    chasing_ball = false;
}

}  // namespace strategy
