#include "offense.hpp"

namespace strategy {

Offense::Offense(int r_id) : Position{r_id, "Offense"}, seeker_{r_id} {}

std::optional<RobotIntent> Offense::derived_get_task(RobotIntent intent) {
    // Get next state, and if different, reset clock
    State new_state = next_state();

    if (current_state_ != new_state) {
        reset_timeout();
        SPDLOG_INFO("Robot {}: now {}", robot_id_, state_to_name(current_state_));
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
            if (has_open_shot() || timed_out()) {
                return SHOOTING_START;
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
            if (has_open_shot() || timed_out()) {
                return SHOOTING_START;
            }

            return POSSESSION;
        }

        case PASSING_START: {
            if (check_is_done()) {
                return PASSING;
            }
            return PASSING_START;
        }

        case PASSING: {
            // If we've finished passing, cool!
            if (check_is_done()) {
                pass_ball(pass_to_robot_id_);
                return DEFAULT;
            }

            // If we didn't successfully pass in time, take a shot
            if (timed_out()) {
                return SHOOTING_START;
            }

            // If we lost the ball completely, give up
            if (distance_to_ball() > kBallTooFarDist) {
                return DEFAULT;
            }

            return PASSING;
        }

        case STEALING: {
            // Go to possession if successful
            if (check_is_done()) {
                return POSSESSION_START;
            }

            // If another robot becomes closer, leave state
            if (!can_steal_ball()) {
                return SEEKING;
            }

            if (timed_out()) {
                // If we timed out and the ball is close, assume we have it
                // (because is_done for settle/collect are not great)
                if (distance_to_ball() < kOwnBallRadius) {
                    return POSSESSION_START;
                } else {
                    return DEFAULT;
                }
            }

            return STEALING;
        }

        case RECEIVING: {
            // If we got it, cool, we have it!
            if (check_is_done() && distance_to_ball() < kOwnBallRadius) {
                return POSSESSION_START;
            }

            // If we failed to get it in time
            if (timed_out()) {
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

        case SHOOTING_START: {
            if (check_is_done()) {
                return SHOOTING;
            }
            if (timed_out()) {
                return DEFAULT;
            }
            return SHOOTING_START;
        }


        case SHOOTING: {
            // If we either succeed or fail, it's time to start over.
            if (check_is_done() || timed_out()) {
                return DEFAULT;
            }

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
            target_ = calculate_best_shot();
            return intent;
        }

        case POSSESSION: {
            return intent;
        }

        case PASSING_START: {
            rj_geometry::Point ball_position = last_world_state_->ball.position;
            auto current_pos = last_world_state_->get_robot(true, robot_id_).pose.position();
            auto move_vector = (current_pos - ball_position).normalized(0.2);

            planning::LinearMotionInstant target{ball_position + move_vector};
            planning::MotionCommand prep_command{"path_target", target, planning::FaceBall{}};

            intent.motion_command = prep_command;

            return intent;
        }

        case PASSING: {
            // Kick to the target robot
            rj_geometry::Point target_robot_pos =
                last_world_state_->get_robot(true, pass_to_robot_id_).pose.position();

            planning::LinearMotionInstant target{target_robot_pos};
            
            auto line_kick_cmd = planning::MotionCommand{"line_kick", planning::LinearMotionInstant{target_}};

            // Set intent to kick
            intent.motion_command = line_kick_cmd;
            intent.shoot_mode = RobotIntent::ShootMode::KICK;
            intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;

            // Adjusts kick speed based on distance.
            // Details: TIGERS 2019 eTDP, rj_gameplay/passer.py
            rj_geometry::Point this_robot_pos =
                last_world_state_->get_robot(true, this->robot_id_).pose.position();

            double dist = target_robot_pos.dist_to(this_robot_pos);
            intent.kick_speed = std::sqrt((std::pow(kFinalBallSpeed, 2)) - (2 * kBallDecel * dist));

            return intent;
        }

        case STEALING: {
            // intercept the ball
            // if ball fast, use settle, otherwise collect
            // if (last_world_state_->ball.velocity.mag() > 0.75) {
            //     auto settle_cmd = planning::MotionCommand{"settle"};
            //     intent.motion_command = settle_cmd;
            //     intent.dribbler_speed = 255.0;
            // } else {

            // rj_geometry::Point increment(0.3, 0.3);
            // auto current_pos = last_world_state_->ball.position - increment;

            // planning::LinearMotionInstant stay_in_place {current_pos};

            // intent.motion_command = planning::MotionCommand{"path_target", stay_in_place,
            // planning::FaceBall{}, false};

            auto collect_cmd = planning::MotionCommand{"collect"};
            intent.motion_command = collect_cmd;
            intent.dribbler_speed = 255.0;
            // }

            return intent;
        }

        case RECEIVING_START: {
            // Turn to face the ball

            auto current_pos = last_world_state_->get_robot(true, robot_id_).pose.position();

            planning::LinearMotionInstant stay_in_place{current_pos};

            intent.motion_command =
                planning::MotionCommand{"path_target", stay_in_place, planning::FaceBall{}};

            return intent;
        }

        case RECEIVING: {
            // intercept the ball
            // if ball fast, use settle, otherwise collect
            // if (last_world_state_->ball.velocity.mag() > 0.75) {
            // auto settle_cmd = planning::MotionCommand{"settle"};
            // intent.motion_command = settle_cmd;
            // intent.dribbler_speed = 255.0;
            // } else {
            auto collect_cmd = planning::MotionCommand{"collect"};
            intent.motion_command = collect_cmd;
            intent.dribbler_speed = 255.0;
            // }

            return intent;
        }

        case SHOOTING_START: {
            // Line kick best shot
            target_ = calculate_best_shot();

            auto line_kick_cmd = planning::MotionCommand{"line_kick_one", planning::LinearMotionInstant{target_}};
            intent.motion_command = line_kick_cmd;
            return intent;
        }

        case SHOOTING: {
            auto line_kick_cmd = planning::MotionCommand{"line_kick_two", planning::LinearMotionInstant{target_}};

            intent.motion_command = line_kick_cmd;
            intent.shoot_mode = RobotIntent::ShootMode::KICK;
            intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
            intent.kick_speed = 4.0;

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

    // If the current robot is far enough away from other robots and there
    // are no other robots
    // in the passing line, process the request Currently, max_receive_distance is used to
    // determine when we are open, but this may need to change
    return (min_robot_dist > max_receive_distance && min_path_dist > max_receive_distance);
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

        if (check_if_open(pass_request->from_robot_id)) response.direct_open = true;

        // SPDLOG_INFO("Robot {} accepts pass", robot_id_);

        comm_response.response = response;
    } else if (const communication::SeekerRequest* seeker_request =
                   std::get_if<communication::SeekerRequest>(&request.request)) {
        if (seeker_request->adding) {
            seeker_points_[seeker_request->robot_id] = rj_geometry::Point{
                seeker_request->seeking_point_x, seeker_request->seeking_point_y};
        } else {
            seeker_points_.erase(seeker_request->robot_id);
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
                current_state_ = PASSING_START;
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
                    // SPDLOG_INFO("Robot {} is sending a pass confirmation", robot_id_);
                    send_pass_confirmation(response.received_robot_ids[i]);
                    // pass_to_robot_id_ = response.received_robot_ids[i];
                    // current_state_ = PASSING_START;
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
    if (current_state_ != SHOOTING) {
        // current_state_ = PASSING_START;
    }
}

void Offense::derived_acknowledge_ball_in_transit() {
    // The ball is coming to me
    current_state_ = RECEIVING;
    reset_timeout();
}

bool Offense::has_open_shot() const {
    // Ball position
    rj_geometry::Point ball_position = this->last_world_state_->ball.position;

    // Goal target location
    rj_geometry::Point best_shot = calculate_best_shot();

    double min_dist = std::numeric_limits<double>::infinity();

    // Vector from target to ball
    rj_geometry::Point ball_to_goal = best_shot - ball_position;
    for (const RobotState& enemy : last_world_state_->their_robots) {
        // Ignore enemies within their defense area
        // this ignores the enemy goalie, which will almost always be in the way of the shot anyway
        if (this->field_dimensions_.their_defense_area().hit(enemy.pose.position())) {
            continue;
        }

        // Vector from enemy to ball
        rj_geometry::Point enemy_vec = enemy.pose.position() - ball_position;

        // I think this means enemy is behind shot (sid)
        if (enemy_vec.dot(ball_to_goal) < 0) {
            continue;
        }

        // Project enemy vector onto our shot line
        auto projection = (enemy_vec.dot(ball_to_goal) / ball_to_goal.dot(ball_to_goal));
        enemy_vec = enemy_vec - (projection)*ball_to_goal;

        // Enemy's distance from our shot line
        double distance = enemy_vec.mag();
        min_dist = std::min(min_dist, distance);
    }

    return min_dist > kEnemyTooCloseRadius;
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
    // Ball location
    rj_geometry::Point ball_position = this->last_world_state_->ball.position;

    // Our robot is closest robot to ball
    bool closest = true;

    auto current_pos = last_world_state_->get_robot(true, robot_id_).pose.position();

    auto our_dist = (current_pos - ball_position).mag();
    for (auto enemy : this->last_world_state_->their_robots) {
        auto dist = (enemy.pose.position() - ball_position).mag();
        if (dist < our_dist) {
            closest = false;
            break;
        }
    }

    if (!closest) {
        return closest;
    }

    for (auto pal : this->last_world_state_->our_robots) {
        // if (pal.robot_id_ == robot_id_) {
        // continue;
        // }
        auto dist = (pal.pose.position() - ball_position).mag();
        if (dist < our_dist) {
            closest = false;
            break;
        }
    }

    return closest;

    // return distance_to_ball() < kStealBallRadius;
}

rj_geometry::Point Offense::calculate_best_shot() const {
    // Goal location
    rj_geometry::Point their_goal_pos = field_dimensions_.their_goal_loc();
    double goal_width = field_dimensions_.goal_width();  // 1.0 meters

    // Ball location
    rj_geometry::Point ball_position = this->last_world_state_->ball.position;

    rj_geometry::Point best_shot = their_goal_pos;
    double best_distance = -1.0;
    rj_geometry::Point increment(0.05, 0);
    rj_geometry::Point curr_point =
        their_goal_pos - rj_geometry::Point(goal_width / 2.0, 0) + increment;
    for (int i = 0; i < 19; i++) {
        double distance = distance_from_their_robots(ball_position, curr_point);
        if (distance > best_distance) {
            best_distance = distance;
            best_shot = curr_point;
        }
        curr_point = curr_point + increment;
    }
    return best_shot;
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
    communication_request_ = communication_request;
}
}  // namespace strategy
