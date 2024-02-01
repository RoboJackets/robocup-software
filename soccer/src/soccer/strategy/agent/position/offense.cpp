#include "offense.hpp"

namespace strategy {

Offense::Offense(int r_id) : Position(r_id) { position_name_ = "Offense"; }

std::optional<RobotIntent> Offense::derived_get_task(RobotIntent intent) {
    State next_state = update_state();

    if (current_state_ != next_state) {
        reset_timeout();
    }

    current_state_ = next_state;

    return state_to_task(intent);
}

Offense::State Offense::update_state() {
    // handle transitions between current state
    WorldState* world_state = this->last_world_state_;

    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = world_state->ball.position;
    double distance_to_ball = robot_position.dist_to(ball_position);

     switch (current_state_) {

        case DEFAULT:

            return SEEKING;

        case SEEKING:

            return SEEKING;

        case POSSESSION_START:

            if (has_open_shot()) {
                return SHOOTING;
            }

            // No open shot, try to pass.
            // This will trigger an automatic switch to passing if a pass is accepted.
            broadcast_direct_pass_request();

            return POSSESSION;

        case POSSESSION:

            if (has_open_shot()) {
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
            if (ball_position.dist_to(robot_position) > kBallTooFarDist) {
                return DEFAULT;
            }


        case STEALING:

            // Go to possession if successful
            if (check_is_done()) {
                return POSSESSION;
            }

            // If we ran out of time or the ball is out of our radius, give up
            if (timed_out() || ball_position.dist_to(robot_position) > kStealBallRadius) {
                return DEFAULT;
            }

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
        
        case SEEKING:

            // Seek

            return std::nullopt; // TODO

        case POSSESSION:

            

            return std::nullopt; // TODO

        case PASSING:

            // Pass to target robot

            return std::nullopt; // TODO

        case STEALING:

            // Settle/Collect

            return std::nullopt; // TODO

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

inline void Offense::reset_timeout() {
    last_time_ = RJ::now();
}

inline bool Offense::timed_out() {
    const auto timeout = timeout(current_state_);
    return timeout >= 0 && RJ::now() - last_time_ > timeout;
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

}  // namespace strategy
