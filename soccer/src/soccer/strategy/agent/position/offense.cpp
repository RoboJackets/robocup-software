#include "offense.hpp"

namespace strategy {

Offense::Offense(int r_id) : Position{r_id}, seeker_{r_id} { position_name_ = "Offense"; }

std::optional<RobotIntent> Offense::derived_get_task(RobotIntent intent) {
    
    // Get next state, and if different, reset clock
    State new_state = next_state();

    if (current_state_ != new_state) {
        reset_timeout();
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
    WorldState* world_state = this->last_world_state_;

    switch (current_state_) {
        case DEFAULT:

            return SEEKING_START;

        case SEEKING_START:

            // Unconditionally only stay in this state for one tick.
            return SEEKING;

        case SEEKING:

            // If the ball seems "stealable", we should switch to STEALING
            if (can_steal_ball()) {
                return STEALING;
            }

            // If we need to get a new seeking target, restart seeking
            if (check_is_done() || last_world_state_->get_robot(true, robot_id_).velocity.linear().mag() <= 0.01) {
                return SEEKING_START;
            }

            return SEEKING;

        case POSSESSION_START:

            // If we can make a shot, take it
            // If we need to stop possessing now, shoot.
            if (has_open_shot() || timed_out()) {
                return SHOOTING;
            }

            // No open shot, try to pass.
            // This will trigger an automatic switch to passing if a pass is accepted.
            broadcast_direct_pass_request();

            return POSSESSION;

        case POSSESSION:

            // If we can make a shot, make it.
            // If we need to stop possessing now, shoot.
            if (has_open_shot() || timed_out()) {
                return SHOOTING;
            }

            return POSSESSION;

        case PASSING:

            // If we've finished passing, cool!
            if (check_is_done()) {
                return DEFAULT;
            }

            // If we didn't successfully pass in time, take a shot
            if (timed_out()) {
                return SHOOTING;
            }

            // If we lost the ball completely, give up
            if (distance_to_ball() > kBallTooFarDist) {
                return DEFAULT;
            }

            return PASSING;

        case STEALING:

            // Go to possession if successful
            if (check_is_done()) {
                return POSSESSION;
            }

            // If we ran out of time or the ball is out of our radius, give up
            if (timed_out() || distance_to_ball() > kBallTooFarDist) {
                return DEFAULT;
            }

            return STEALING;

        case RECEIVING:

            // If we got it, cool, we have it!
            if (check_is_done()) {
                return POSSESSION;
            }

            // If we failed to get it in time
            if (timed_out()) {
                return DEFAULT;
            }

        case SHOOTING:

            // If we either succeed or fail, it's time to start over.
            if (check_is_done() || timed_out()) {
                return DEFAULT;
            }

            return DEFAULT;
    }
}

std::optional<RobotIntent> Offense::state_to_task(RobotIntent intent) {
    switch (current_state_) {
        case DEFAULT:
            // Do nothing: empty motion command
            intent.motion_command = planning::MotionCommand{};
            return intent;

        case SEEKING_START:

            // Calculate a new seeking point
            seeker_.reset_target();
            return seeker_.get_task(std::move(intent), last_world_state_, field_dimensions_);

        case SEEKING:

            return seeker_.get_task(std::move(intent), last_world_state_, field_dimensions_);

        case POSSESSION:

            return std::nullopt;  // TODO

        case PASSING:

            // Pass to target robot

            return std::nullopt;  // TODO

        case STEALING:

            // Settle/Collect

            return std::nullopt;  // TODO

        case RECEIVING:

            // Settle/Collect but like slower

            return std::nullopt;

        case SHOOTING:

            // Line kick

            return std::nullopt;
    }
}

communication::PosAgentResponseWrapper Offense::receive_communication_request(
    communication::AgentPosRequestWrapper request) {
    // PassRequests: only in offense right now
    if (const communication::PassRequest* pass_request =
            std::get_if<communication::PassRequest>(&request.request)) {
        // If the robot recieves a PassRequest, only process it if we are oppen

        rj_geometry::Point robot_position =
            last_world_state_->get_robot(true, robot_id_).pose.position();
        rj_geometry::Point from_robot_position =
            last_world_state_->get_robot(true, pass_request->from_robot_id).pose.position();
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

        // If the current robot is far enough away from other robots and there are no other robots
        // in the passing line, process the request Currently, max_receive_distance is used to
        // determine when we are open, but this may need to change
        if (min_robot_dist > max_receive_distance && min_path_dist > max_receive_distance) {
            communication::PassResponse response = Position::receive_pass_request(*pass_request);
            return {response};
        }

        return {};
    } else {
        // Super: other kinds of requests
        return Position::receive_communication_request(request);
    }
}

void Offense::derived_acknowledge_pass() {
    // I have been chosen as the receiver
    current_state_ = RECEIVING;
}

void Offense::derived_pass_ball() {
    // When we have the ball we send out a pass request.
    // However, if we've since started shooting, just do that.
    // Otherwise, we can now pass because somebody has accepted our pass.
    if (current_state_ != SHOOTING) {
        current_state_ = PASSING;
    }
}

void Offense::derived_acknowledge_ball_in_transit() {
    // The ball is coming to me
    current_state_ = RECEIVING;
}

bool Offense::has_open_shot() const {
    // Goal location
    rj_geometry::Point their_goal_pos = field_dimensions_.their_goal_loc();
    double goal_width = field_dimensions_.goal_width();  // 1.0 meters

    // Ball location
    rj_geometry::Point ball_position = this->last_world_state_->ball.position;

    double best_distance = -1.0;
    rj_geometry::Point increment(0.05, 0);
    rj_geometry::Point curr_point =
        their_goal_pos - rj_geometry::Point(goal_width / 2.0, 0) + increment;
    for (int i = 0; i < 19; i++) {
        double distance = distance_from_their_robots(ball_position, curr_point);
        if (distance > best_distance) {
            best_distance = distance;
        }
        curr_point = curr_point + increment;
    }

    return best_distance > max_receive_distance;
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
    return distance_to_ball() < kStealBallRadius;
}

}  // namespace strategy
