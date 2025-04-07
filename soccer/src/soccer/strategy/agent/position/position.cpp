#include "position.hpp"

#include <limits>

#include "game_state.hpp"

namespace strategy {

Position::Position(int r_id) : robot_id_(r_id) {}

Position::Position(int r_id, std::string position_name)
    : position_name_{std::move(position_name)}, robot_id_{r_id} {};

std::optional<RobotIntent> Position::get_task(WorldState& world_state,
                                              FieldDimensions& field_dimensions,
                                              PlayState& play_state) {
    // Point class variables to parameter references
    // TODO (Prabhanjan): Don't copy references into local vars
    field_dimensions_ = field_dimensions;
    last_world_state_ = &world_state;

    // init an intent with our robot id
    RobotIntent intent = RobotIntent{};
    intent.robot_id = robot_id_;

    // if world_state invalid, return empty MotionCommand (equivalent to HALT)
    if (!assert_world_state_valid()) {
        intent.motion_command = planning::MotionCommand{};
        return intent;
    }
    // delegate to derived class to complete behavior
    return derived_get_task(intent);
}

void Position::set_time_left(double time_left) { time_left_ = time_left; }

void Position::set_is_done() { is_done_ = true; }

void Position::set_goal_canceled() { goal_canceled_ = true; }

void Position::set_goalie_id(int goalie_id) { goalie_id_ = goalie_id; }

bool Position::check_is_done() {
    if (is_done_) {
        is_done_ = false;
        return true;
    }
    return false;
}

bool Position::check_goal_canceled() {
    if (goal_canceled_) {
        goal_canceled_ = false;
        return true;
    }
    return false;
}

void Position::update_play_state(const PlayState& play_state) { current_play_state_ = play_state; }

void Position::update_field_dimensions(const FieldDimensions& field_dims) {
    field_dimensions_ = field_dims;
}

void Position::update_alive_robots(std::array<bool, kNumShells> alive_robots) {
    alive_robots_ = alive_robots;

    if (alive && !alive_robots_[robot_id_]) {
        alive = false;
        die();
    } else if (!alive && alive_robots_[robot_id_]) {
        alive = true;
        revive();
    }
}

bool Position::assert_world_state_valid() {
    if (last_world_state_ == nullptr) {
        SPDLOG_WARN("WorldState!");
        return false;
    }
    return true;
}

std::deque<communication::PosAgentRequestWrapper> Position::send_communication_request() {
    // Return and reset this member
    return std::exchange(communication_requests_, {});
}

void Position::receive_communication_response(communication::AgentPosResponseWrapper response) {
    for (u_int32_t i = 0; i < response.responses.size(); i++) {
        if (const communication::Acknowledge* acknowledge =
                std::get_if<communication::Acknowledge>(&response.responses[i])) {
            // if the acknowledgement is from an incoming pass request -> pass the ball
            if (const communication::IncomingBallRequest* incoming_ball_request =
                    std::get_if<communication::IncomingBallRequest>(&response.associated_request)) {
                // SPDLOG_INFO("Robot {} received incoming ball request", robot_id_);
                pass_ball(response.received_robot_ids[i]);
            }

        } else if (const communication::PassResponse* pass_response =
                       std::get_if<communication::PassResponse>(&response.responses[i])) {
            // get the associated pass request for this response
            // SPDLOG_INFO("Robot {} receives pass response", robot_id_);
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
                }
            }
        }
    }
}

communication::PosAgentResponseWrapper Position::receive_communication_request(
    communication::AgentPosRequestWrapper request) {
    communication::PosAgentResponseWrapper comm_response{};
    if (const communication::PassRequest* pass_request =
            std::get_if<communication::PassRequest>(&request.request)) {
        // Pass is needed. respond if open
        communication::PassResponse pass_response = receive_pass_request(*pass_request);
        pass_response.direct_open = false;
        comm_response.response = pass_response;

    } else if (const communication::IncomingBallRequest* incoming_ball_request =
                   std::get_if<communication::IncomingBallRequest>(&request.request)) {
        // I have been chosen. Offense: RECEVING_START (used to be called FACING)
        communication::Acknowledge incoming_pass_acknowledge =
            acknowledge_pass(*incoming_ball_request);
        // SPDLOG_INFO("Robot {} acknowledges incoming ball request", robot_id_);
        comm_response.response = incoming_pass_acknowledge;
    } else if (const communication::BallInTransitRequest* ball_in_transit_request =
                   std::get_if<communication::BallInTransitRequest>(&request.request)) {
        communication::Acknowledge ball_in_transit_acknowledge =

            // Pass has started. Offense: RECEIVING
            acknowledge_ball_in_transit(*ball_in_transit_request);
        // SPDLOG_INFO("Robot {} acknowledges ball in transit request", robot_id_);
        comm_response.response = ball_in_transit_acknowledge;
    } else {
        communication::Acknowledge acknowledge{};
        communication::generate_uid(acknowledge);
        comm_response.response = acknowledge;
    }

    return comm_response;
}

const std::string Position::get_name() { return position_name_; }

void Position::send_direct_pass_request(std::vector<u_int8_t> target_robots) {
    communication::PassRequest pass_request{};
    communication::generate_uid(pass_request);
    pass_request.direct = true;
    pass_request.from_robot_id = robot_id_;

    communication::PosAgentRequestWrapper communication_request{};
    communication_request.request = pass_request;
    communication_request.target_agents = target_robots;
    communication_request.urgent = true;
    communication_request.broadcast = false;
    communication_requests_.push_back(communication_request);
}

void Position::broadcast_direct_pass_request() {
    communication::PassRequest pass_request{};
    communication::generate_uid(pass_request);
    pass_request.direct = true;
    pass_request.from_robot_id = robot_id_;

    communication::PosAgentRequestWrapper communication_request{};
    communication_request.request = pass_request;
    communication_request.urgent = false;
    communication_request.broadcast = true;
    communication_requests_.push_back(communication_request);
}

void Position::broadcast_kicker_request() {
    communication::KickerRequest kicker_request{};
    communication::generate_uid(kicker_request);
    kicker_request.robot_id = robot_id_;

    double distance;

    if (kicker_distances_.count(robot_id_)) {
        distance = kicker_distances_[robot_id_];
    } else if (!last_world_state_) {
        distance = std::numeric_limits<double>::infinity();
    } else {
        distance = last_world_state_->ball.position.dist_to(
            last_world_state_->get_robot(true, robot_id_).pose.position());
    }

    // if (last_world_state_) {
    //     distance = last_world_state_->ball.position.dist_to(
    //         last_world_state_->get_robot(true, robot_id_).pose.position());
    // }

    kicker_distances_[robot_id_] = distance;
    kicker_request.distance = distance;

    communication::PosAgentRequestWrapper communication_request{};
    communication_request.request = kicker_request;
    communication_request.urgent = false;
    communication_request.broadcast = true;
    communication_requests_.push_back(communication_request);
}

communication::PassResponse Position::receive_pass_request(
    communication::PassRequest pass_request) {
    communication::PassResponse pass_response{};
    communication::generate_uid(pass_response);

    if (pass_request.direct) {
        // Handle direct pass request
        pass_response.direct_open = true;
    } else {
        // TODO: Handle indirect pass request
        pass_response.direct_open = false;
    }

    return pass_response;
}

void Position::send_pass_confirmation(u_int8_t target_robot) {
    communication::IncomingBallRequest incoming_ball_request{};
    incoming_ball_request.from_robot_id = robot_id_;
    communication::generate_uid(incoming_ball_request);

    communication::PosAgentRequestWrapper communication_request{};
    communication_request.request = incoming_ball_request;
    communication_request.target_agents = {target_robot};
    communication_request.broadcast = false;
    communication_request.urgent = true;

    communication_requests_.push_back(communication_request);
}

communication::Acknowledge Position::acknowledge_pass(
    communication::IncomingBallRequest incoming_ball_request) {
    communication::Acknowledge acknowledge_response{};
    communication::generate_uid(acknowledge_response);

    face_robot_id = incoming_ball_request.from_robot_id;

    derived_acknowledge_pass();

    return acknowledge_response;
}

void Position::pass_ball(int robot_id) {
    target_robot_id = robot_id;

    communication::BallInTransitRequest ball_in_transit_request{};
    ball_in_transit_request.from_robot_id = robot_id_;
    communication::generate_uid(ball_in_transit_request);

    communication::PosAgentRequestWrapper communication_request{};
    communication_request.request = ball_in_transit_request;
    communication_request.target_agents = {(u_int8_t)robot_id};
    communication_request.urgent = true;
    communication_request.broadcast = false;

    communication_requests_.push_back(communication_request);

    derived_pass_ball();
}

double Position::distance_from_their_robots(rj_geometry::Point tail, rj_geometry::Point head) const {
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

rj_geometry::Point Position::calculate_best_shot() const {
    rj_geometry::Point their_goal_pos = field_dimensions_.their_goal_loc();

    return their_goal_pos;

    double goal_width = field_dimensions_.goal_width();
    double goal_y = their_goal_pos.y();
    rj_geometry::Point left_goal_post = field_dimensions_.their_left_goal_post_coordinate();
    rj_geometry::Point right_goal_post = field_dimensions_.their_right_goal_post_coordinate();

    // Using robot position since the ball might be pivoted
    rj_geometry::Point ball_position = last_world_state_->get_robot(true, robot_id_).pose.position();

    // Compute angles from the ball to the posts relative to vertical
    double phi_left  = std::atan2(left_goal_post.x() - ball_position.x(), goal_y - ball_position.y());
    double phi_right = std::atan2(right_goal_post.x() - ball_position.x(), goal_y - ball_position.y());

    // Calculate distances from the ball to each post
    double d_left  = (ball_position - left_goal_post).mag();
    double d_right = (ball_position - right_goal_post).mag();

    // Adjust the angles to account for the ball's radius (x2 for extra margin)
    double safe_phi_left  = phi_left + std::asin((2 * kBallRadius) / d_left);
    double safe_phi_right = phi_right - std::asin((2 * kBallRadius) / d_right);

    // Project the safe angles onto the goal line to determine safe x–coordinates
    double safe_x_left  = ball_position.x() + (goal_y - ball_position.y()) * std::tan(safe_phi_left);
    double safe_x_right = ball_position.x() + (goal_y - ball_position.y()) * std::tan(safe_phi_right);
    
    // Set up the safe interval along the goal line and iterate through candidate shot points
    rj_geometry::Point start_point(safe_x_left, goal_y);
    rj_geometry::Point end_point(safe_x_right, goal_y);
    rj_geometry::Point best_shot = their_goal_pos;
    double best_distance = -std::numeric_limits<double>::infinity();
    rj_geometry::Point increment = (end_point - start_point) / (kShotPoints - 1);

    SPDLOG_INFO("Robot {}: range: {} to {}", robot_id_, safe_x_left, safe_x_right);

    for (int i = 0; i < kShotPoints; i++) {
        rj_geometry::Point curr_point = start_point + i * increment;
        double distance = distance_from_their_robots(ball_position, curr_point);
        if (distance > best_distance) {
            best_distance = distance;
            best_shot = curr_point;
        }
    }
    SPDLOG_INFO("Robot {}: best shot: {}, {}", robot_id_, best_shot.x(), best_shot.y());
    return best_shot;
}

communication::Acknowledge Position::acknowledge_ball_in_transit(
    communication::BallInTransitRequest ball_in_transit_request) {
    communication::Acknowledge acknowledge_response{};
    communication::generate_uid(acknowledge_response);

    face_robot_id = ball_in_transit_request.from_robot_id;

    derived_acknowledge_ball_in_transit();

    return acknowledge_response;
}

}  // namespace strategy
