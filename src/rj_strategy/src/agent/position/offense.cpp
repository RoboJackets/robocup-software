#include "rj_strategy/agent/position/offense.hpp"

namespace strategy {

Offense::Offense(int r_id) : Position{r_id, "Offense"}, seeker_{r_id} {}

Offense::Offense(Position&& other) : Position{std::move(other)}, seeker_{robot_id_} {
    position_name_ = "Offense";
}

std::optional<RobotIntent> Offense::derived_get_task(RobotIntent intent) {
    // Get next state, and if different, reset clock
    State new_state = next_state();

    if (current_state_ != new_state) {
        reset_timeout();

        if (current_state_ == SEEKING) {
            broadcast_seeker_request(rj_geometry::Point{}, false);
        }
    }

    current_state_ = new_state;

    // Calculate task based on state
    return state_to_task(intent);
}

std::string Offense::get_current_state() {
    return std::string{"Offense"} + std::to_string(static_cast<int>(current_state_));
}

Offense::State Offense::next_state() {
    // handle transitions between current state
    switch (current_state_) {
        case DEFAULT: {
            return SEEKING_START;
        }

        case SEEKING_START: {
            // Unconditionally only stay in this state for one tick.
            return SEEKING;
        }

        case SEEKING: {
            // If the ball seems "stealable", we should switch to STEALING
            if (can_steal_ball()) {
                return STEALING;
            }

            // If we need to get a new seeking target, restart seeking
            if (check_is_done() ||
                last_world_state_->get_robot(true, robot_id_).velocity.linear().mag() <= 0.01) {
                return SEEKING_START;
            }

            return SEEKING;
        }
        case POSSESSION_START: {
            // If we can make a shot, take it
            // If we need to stop possessing now, shoot.
            if (!can_steal_ball()) {
                return SEEKING_START;
            }
            if (has_open_shot() || timed_out()) {
                target_ = calculate_best_shot(last_world_state_, field_dimensions_);
                return SHOOTING;
            }

            // No open shot, try to pass.
            // This will trigger an automatic switch to passing if a pass is
            // accepted.
            broadcast_direct_pass_request();

            return POSSESSION;
        }

        case POSSESSION: {
            // If we can make a shot, make it.
            // If we need to stop possessing now, shoot.
            if (!can_steal_ball()) {
                return SEEKING_START;
            }

            if (has_open_shot() || timed_out()) {
                target_ = calculate_best_shot(last_world_state_, field_dimensions_);
                return SHOOTING;
            }

            broadcast_direct_pass_request();

            return POSSESSION;
        }

        case PASSING: {
            if (!can_steal_ball() || (can_steal_ball() && has_open_shot())) {
                send_kick_failed_to_receiver(pass_to_robot_id_);
                return DEFAULT;
            }
            if (check_is_done()) {
                pass_ball(pass_to_robot_id_);
                return PASSING_FINISHED;
            }

            return PASSING;
        }

        case PASSING_FINISHED: {
            // Failsafe: if the ball is still near the passer after the kick
            // failsafe window, the kick failed — notify the receiver and reset.
            if (kick_failed()) {
                send_kick_failed_to_receiver(pass_to_robot_id_);
                return DEFAULT;
            }

            // Wait for PassReceivedRequest from receiver (handled in
            // receive_communication_request) or timeout.
            if (timed_out()) {
                return DEFAULT;
            }
            return PASSING_FINISHED;
        }

        case STEALING: {
            // Go to possession if successful
            // if (check_is_done() || distance_to_ball() < kOwnBallRadius) {
                return POSSESSION_START;
            // }

            // If another robot becomes closer, leave state
            if (!can_steal_ball()) {
                return SEEKING_START;
            }

            if (timed_out()) {
                return DEFAULT;
            }

            return STEALING;
        }

        case RECEIVING: {
            // If we got it, cool, we have it!
            if (check_is_done() && can_steal_ball()) {
                send_pass_received_to_passer(face_robot_id);
                return POSSESSION_START;
            }

            // If we failed to get it in time or if ball is out of reach
            if (!ball_in_play_area(last_world_state_, field_dimensions_) || timed_out()) {
                send_pass_received_to_passer(face_robot_id);
                return DEFAULT;
            }

            return RECEIVING;
        }

        case RECEIVING_START: {
            // Stay in this state until either:
            // a) Incoming Ball Request received
            // b) Timed out
            // both of which are handled in other member functions
            return RECEIVING_START;
        }

        case SHOOTING: {
            // timed_out() guards against getting stuck lined up on the ball
            // without the break beam ever triggering a kick. Without it, none of
            // the other conditions become true while parked on the ball, so the
            // robot would stand still in SHOOTING forever.
            if (!ball_in_play_area(last_world_state_, field_dimensions_) || check_is_done() ||
                !has_open_shot() || !can_steal_ball() || timed_out()) {
                return DEFAULT;
            }
            // if (distance_to_ball() > kOwnBallRadius) {
            //     return DEFAULT;
            // }
            return SHOOTING;
        }
    }
}

std::optional<RobotIntent> Offense::state_to_task(RobotIntent intent) {
    switch (current_state_) {
        case DEFAULT: {
            // Do nothing: empty motion command
            intent.motion_command = planning::MotionCommand{};
            return intent;
        }

        case SEEKING_START: {
            // Calculate a new seeking point
            seeker_.reset_target();
            seeker_.set_seeker_points(seeker_points_);
            std::optional<RobotIntent> actual_intent =
                seeker_.get_task(std::move(intent), last_world_state_, field_dimensions_);
            broadcast_seeker_request(seeker_.get_target_point(), true);
            return actual_intent;
        }

        case SEEKING: {
            return seeker_.get_task(std::move(intent), last_world_state_, field_dimensions_);
        }

        case POSSESSION_START: {
            target_ = calculate_best_shot(last_world_state_, field_dimensions_);
            intent.motion_command = planning::MotionCommand{};

            return intent;
        }

        case POSSESSION: {
            intent.motion_command = planning::MotionCommand{};

            return intent;
        }

        case PASSING: {
            auto vel =
                last_world_state_->get_robot(true, pass_to_robot_id_).velocity.linear().mag();
            if (vel > 0.15) {
                return intent;
            }
            rj_geometry::Point target_robot_pos =
                last_world_state_->get_robot(true, pass_to_robot_id_).pose.position();
            planning::LinearMotionInstant target{target_robot_pos};
            auto pivot_cmd =
                planning::MotionCommand{"line_kick", target, planning::FaceTarget{}, true};
            intent.motion_command = pivot_cmd;
            intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;

            // Adjusts kick speed based on distance.
            // Details: TIGERS 2019 eTDP, rj_gameplay/passer.py
            rj_geometry::Point this_robot_pos =
                last_world_state_->get_robot(true, this->robot_id_).pose.position();

            intent.kick_speed = calculate_kick_speed(target_robot_pos.dist_to(this_robot_pos), 0.0);

            return intent;
        }

        case PASSING_FINISHED: {
            // Stay in place until we receive PassReceivedRequest from receiver.
            intent.motion_command = planning::MotionCommand{};
            return intent;
        }

        case STEALING: {
            // Approach the ball from the side away from the opponent's goal so
            // the ball ends up between us and the goal, leaving us set up to
            // push it goalward. Target a point a fixed distance behind the ball
            // along the goal->ball line.
            rj_geometry::Point ball_position = last_world_state_->ball.position;
            rj_geometry::Point their_goal = field_dimensions_.their_goal_loc();
            rj_geometry::Point goal_to_ball = (ball_position - their_goal).normalized();
            rj_geometry::Point steal_point =
                ball_position + goal_to_ball * kStealApproachDistance;

            auto collect_cmd = planning::MotionCommand{
                "path_target", planning::LinearMotionInstant{steal_point}, planning::FaceBall{}};
            intent.motion_command = collect_cmd;

            return intent;
        }

        case RECEIVING_START: {
            // Turn to face the ball

            return seeker_.get_task(std::move(intent), last_world_state_, field_dimensions_);
        }

        case RECEIVING: {
            // intercept the ball
            // if ball fast, use settle, otherwise collect
            // if (last_world_state_->ball.velocity.mag() > 0.75) {
            // auto settle_cmd = planning::MotionCommand{"settle"};
            // intent.motion_command = settle_cmd;
            // intent.dribbler_speed = 255.0;
            // } else {
            return seeker_.get_task(std::move(intent), last_world_state_, field_dimensions_);
        }

        case SHOOTING: {
            // link kick because collect is garbage
            planning::LinearMotionInstant target{
                calculate_best_shot(last_world_state_, field_dimensions_)};

            auto shoot_cmd =
                planning::MotionCommand{"line_kick", target, planning::FaceTarget{}, true};
            intent.motion_command = shoot_cmd;
            intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
            intent.kick_speed = max_kick_speed();  // Integer value in [0,15]
            return intent;
        }
    }
}

bool Offense::check_if_open(int target_robot_shell) {
    rj_geometry::Point robot_position =
        last_world_state_->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point from_robot_position =
        last_world_state_->get_robot(true, target_robot_shell).pose.position();
    rj_geometry::Segment pass_path{from_robot_position, robot_position};
    double min_robot_dist = 10000;
    float min_path_dist = 10000;

    // Calculates the minimum distance from the current robot to all other robots
    // Also calculates the minimum distance from another robot to the passing line
    for (auto bot : last_world_state_->their_robots) {
        rj_geometry::Point opp_pos = bot.pose.position();
        min_robot_dist = std::min(min_robot_dist, robot_position.dist_to(opp_pos));
        min_path_dist = std::min(min_path_dist, pass_path.dist_to(opp_pos));
    }

    min_robot_dist = std::min(min_robot_dist, robot_position.dist_to(from_robot_position));

    // If the current robot is far enough away from other robots and there
    // are no other robots
    // in the passing line, process the request Currently, max_receive_distance is used to
    // determine when we are open, but this may need to change
    // /2 is there to help create more leniency when passing
    return (min_robot_dist > max_receive_distance && min_path_dist > max_receive_distance / 2);
}

communication::PosAgentResponseWrapper Offense::receive_communication_request(
    communication::AgentPosRequestWrapper request) {
    communication::PosAgentResponseWrapper comm_response =
        Position::receive_communication_request(request);

    // PassRequests: only in offense right now
    if (const communication::PassRequest* pass_request =
            std::get_if<communication::PassRequest>(&request.request)) {
        // If the robot recieves a PassRequest, only process it if we are open

        auto response = Position::receive_pass_request(*pass_request);
        response.direct_open = true;

        comm_response.response = response;
    } else if (const communication::SeekerRequest* seeker_request =
                   std::get_if<communication::SeekerRequest>(&request.request)) {
        if (seeker_request->adding) {
            seeker_points_[seeker_request->robot_id] = rj_geometry::Point{
                seeker_request->seeking_point_x, seeker_request->seeking_point_y};
        } else {
            seeker_points_.erase(seeker_request->robot_id);
        }
    } else if (std::get_if<communication::PassReceivedRequest>(&request.request)) {
        // Receiver has controlled the ball, we can leave PASSING_FINISHED.
        if (current_state_ == PASSING_FINISHED) {
            current_state_ = DEFAULT;
        }
        // Kick failed: passer notifies receiver to abort receiving.
        if (current_state_ == RECEIVING || current_state_ == RECEIVING_START) {
            current_state_ = DEFAULT;
        }
    }

    return comm_response;
}

// Receiving a response. THis means we initiated a request earlier
void Offense::receive_communication_response(communication::AgentPosResponseWrapper response) {
    for (u_int32_t i = 0; i < response.responses.size(); i++) {
        if (const communication::Acknowledge* acknowledge =
                std::get_if<communication::Acknowledge>(&response.responses[i])) {
            // if the acknowledgement is from an incoming pass request -> pass the ball
            if (const communication::IncomingBallRequest* incoming_ball_request =
                    std::get_if<communication::IncomingBallRequest>(&response.associated_request)) {
                // SPDLOG_INFO("Robot {} received incoming ball request",
                // robot_id_);

                // Chosen Robot has told us they are ready to receive
                current_state_ = PASSING;
                pass_to_robot_id_ = response.received_robot_ids[i];

                // pass_ball(response.received_robot_ids[i]);
            }

        } else if (const communication::PassResponse* pass_response =
                       std::get_if<communication::PassResponse>(&response.responses[i])) {
            // get the associated pass request for this response
            // SPDLOG_INFO("Robot {} receives pass response", robot_id_);

            // Robot has told us they are open
            if (const communication::PassRequest* sent_pass_request =
                    std::get_if<communication::PassRequest>(&response.associated_request)) {
                // SPDLOG_INFO(
                // "Robot {} found associated request from {}: direct: {}, direct_open: {}",
                // robot_id_, response.received_robot_ids[i], sent_pass_request->direct,
                // pass_response->direct_open);

                if (sent_pass_request->direct && pass_response->direct_open) {
                    // if direct -> pass to first robot
                    SPDLOG_INFO("Robot {} is sending a pass confirmation", robot_id_);
                    send_pass_confirmation(response.received_robot_ids[i]);
                    // pass_to_robot_id_ = response.received_robot_ids[i];
                    // current_state_ = PASSING;
                }
            }
        }
    }
}

void Offense::derived_acknowledge_pass() {
    // I have been chosen as the receiver
    current_state_ = RECEIVING_START;
    reset_timeout();
}

void Offense::derived_pass_ball() {
    // When we have the ball we send out a pass request.
    // However, if we've since started shooting, just do that.
    // Otherwise, we can now pass because somebody has accepted our pass.
    // if (current_state_ != SHOOTING) {
    // current_state_ = PASSING;
    // }
}

void Offense::derived_acknowledge_ball_in_transit() {
    // The ball is coming to me
    current_state_ = RECEIVING;
    reset_timeout();
}

bool Offense::has_open_shot() const {
    rj_geometry::Point best_shot = calculate_best_shot(last_world_state_, field_dimensions_);
    double clearance_angle =
        shot_clearance(last_world_state_->ball.position, best_shot, last_world_state_);
    return clearance_angle >=
           0.05;  // if there's >= 3 degrees (0.05 radians) of clearance, that's an open shot
}


double Offense::distance_from_their_robots(rj_geometry::Point tail, rj_geometry::Point head) const {
    rj_geometry::Point vec = head - tail;
    auto& their_robots = this->last_world_state_->their_robots;

    double min_angle = -0.5;
    for (auto enemy : their_robots) {
        rj_geometry::Point enemy_vec = enemy.pose.position() - tail;
        if (enemy_vec.dot(vec) < 0) {
            continue;
        }
        auto projection = (enemy_vec.dot(vec) / vec.dot(vec));
        enemy_vec = enemy_vec - (projection)*vec;
        double distance = enemy_vec.mag();
        if (distance < (kRobotRadius + kBallRadius)) {
            return -1.0;
        }
        double angle = distance / projection;
        if ((min_angle < 0) || (angle < min_angle)) {
            min_angle = angle;
        }
    }
    return min_angle;
}

bool Offense::can_steal_ball() const {
    // If ball is not legally accessible, obviously can't steal
    if (!ball_in_play_area(last_world_state_, field_dimensions_)) {
        return false;
    }
    // Ball location
    rj_geometry::Point ball_position = this->last_world_state_->ball.position;

    // Our robot is closest robot to ball
    bool closest = true;

    auto current_pos = last_world_state_->get_robot(true, robot_id_).pose.position();

    auto our_dist = (current_pos - ball_position).mag();

    for (auto pal : this->last_world_state_->our_robots) {
        auto dist = (pal.pose.position() - ball_position).mag();
        if (dist < our_dist) {
            closest = false;
            break;
        }
    }
    return closest;
}

bool Offense::kick_failed() const {
    return (last_time_ + kKickFailsafeTimeout < RJ::now()) && (distance_to_ball() < kOwnBallRadius);
}

void Offense::send_kick_failed_to_receiver(u_int8_t receiver_robot_id) {
    communication::PassReceivedRequest pass_received_request{};
    pass_received_request.from_robot_id = robot_id_;
    communication::generate_uid(pass_received_request);

    communication::PosAgentRequestWrapper communication_request{};
    communication_request.request = pass_received_request;
    communication_request.target_agents = {receiver_robot_id};
    communication_request.broadcast = false;
    communication_request.urgent = true;
    communication_requests_.push_back(communication_request);
}

void Offense::broadcast_seeker_request(rj_geometry::Point seeking_point, bool adding) {
    communication::SeekerRequest seeker_request{};
    communication::generate_uid(seeker_request);
    seeker_request.robot_id = robot_id_;
    seeker_request.seeking_point_x = seeking_point.x();
    seeker_request.seeking_point_y = seeking_point.y();
    seeker_request.adding = adding;

    communication::PosAgentRequestWrapper communication_request{};
    communication_request.request = seeker_request;
    communication_request.urgent = false;
    communication_request.broadcast = true;
    communication_requests_.push_back(communication_request);
}


}  // namespace strategy
