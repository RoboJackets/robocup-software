#include "defense.hpp"

namespace strategy {

Defense::Defense(int r_id) : Position(r_id, "Defense"), marker_{field_dimensions_} {
}

Defense::Defense(const Position& other) : Position{other}, marker_{field_dimensions_} {
    position_name_ = "Defense";
    walling_robots_ = {};
}

std::optional<RobotIntent> Defense::derived_get_task(RobotIntent intent) {
    // SPDLOG_INFO("waller length (sus) {}, {}", walling_robots_.size(), robot_id_);
    current_state_ = update_state();
    // waller_id_ = get_waller_id();
    return state_to_task(intent);
}

std::string Defense::get_current_state() {
    return std::string{"Defense"} + std::to_string(static_cast<int>(current_state_));
}

Defense::State Defense::update_state() {
    // Waller comms
    if (current_state_ != WALLING && current_state_ != JOINING_WALL && waller_id_ != -1) {
        send_leave_wall_request();
        walling_robots_ = {(u_int8_t)robot_id_};
        waller_id_ = -1;
    } // I'm pretty sure some of our changes don't respect waller_id_, but it seems to work :/

    // General
    rj_geometry::Point my_robot_position = this->last_world_state_->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = this->last_world_state_->ball.position;
    double my_distance_to_ball = my_robot_position.dist_to(ball_position);


    switch (current_state_) {
        case IDLING: { // Dead state
            return IDLING;
        }
        case JOINING_WALL: { // Unconditional comms skip state
            send_join_wall_request();
            walling_robots_ = {(u_int8_t)robot_id_};
            return WALLING;
        }
        case WALLING: { // Wall defense
            if (ball_in_red() || we_in_red()) { // BAD BAD FIX
                return WALLING;
            }

            // If we are the closest robot on the field to the ball, break from the wall and go for a shot.
            if (we_are_closer_than_teammates() && we_are_closer_than_enemies()) {
                SPDLOG_INFO("Robot {} moving on the attack.", robot_id_);
                return WALLER_STEAL;
            }
        
            return WALLING;
        }
        case WALLER_STEAL: { // Begin approach
            // If our fast break fails and we're no longer the closest, go back to the wall.
            if (!we_are_closer_than_enemies()) {
                return JOINING_WALL;
            }
            // If, for some reason, two teammates are both chasing the ball and we're further, go back to the wall.
            if (!we_are_closer_than_teammates()) {
                return JOINING_WALL;
            }
            // If ball is inaccessible (or we have chased it to an illegal pose), go back to wall.
            if (ball_in_red() || we_in_red()) { 
                return JOINING_WALL;
            }
            // If in position, fire away.
            if (check_is_done()) { 
                shot_target_ = calculate_best_shot();
                return KICK;
            }

            return WALLER_STEAL;
        }   
        case KICK: { // Kick that ball
            if (ball_in_red() || we_in_red() || check_is_done()) {
                return JOINING_WALL;
            }
            
            return KICK;
        }
    }

    // Failthrough (shouldn't happen)
    return current_state_;
}

std::optional<RobotIntent> Defense::state_to_task(RobotIntent intent) {
    switch (current_state_) {
        case IDLING: {
            auto empty_motion_cmd = planning::MotionCommand{};
            intent.motion_command = empty_motion_cmd;
            return intent;
        }
        case JOINING_WALL: { // comms only state
            return std::nullopt;
        }
        case WALLING: {
            if (!walling_robots_.empty() && waller_id_ != -1) {
                Waller waller{waller_id_, walling_robots_};
                return waller.get_task(intent, this->last_world_state_, this->field_dimensions_);
            }
        }
        case WALLER_STEAL: {
            planning::LinearMotionInstant target{field_dimensions_.their_goal_loc()};
            auto pivot_cmd = planning::MotionCommand{"line_pivot", target, planning::FaceTarget{}, false, this->last_world_state_->ball.position};
            pivot_cmd.pivot_radius = kRobotRadius * 2.0;
            intent.motion_command = pivot_cmd;
            
            return intent;
        }
        case KICK: {
            auto line_kick_cmd = planning::MotionCommand{"line_kick", planning::LinearMotionInstant{shot_target_}};
            intent.motion_command = line_kick_cmd;
            intent.shoot_mode = RobotIntent::ShootMode::KICK;
            intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
            intent.kick_speed = 5.0;
            
            return intent;
        }
    }

    // Failthrough (shouldn't happen)
    return std::nullopt;
}

void Defense::receive_communication_response(communication::AgentPosResponseWrapper response) {
    // Call to super
    Position::receive_communication_response(response);

    // Handle join wall response
    if (const communication::JoinWallRequest* join_request =
            std::get_if<communication::JoinWallRequest>(&response.associated_request)) {
        for (communication::AgentResponseVariant response : response.responses) {
            if (const communication::JoinWallResponse* join_response =
                    std::get_if<communication::JoinWallResponse>(&response)) {
                handle_join_wall_response(*join_response);
            }
        }
    }
}

communication::PosAgentResponseWrapper Defense::receive_communication_request(
    communication::AgentPosRequestWrapper request) {
    // Call to super
    communication::PosAgentResponseWrapper response =
        Position::receive_communication_request(request);

    // Handle join and leave wall request
    if (const communication::JoinWallRequest* join_request =
            std::get_if<communication::JoinWallRequest>(&request.request)) {
        response.response = handle_join_wall_request(*join_request);
    } else if (const communication::LeaveWallRequest* leave_request =
                   std::get_if<communication::LeaveWallRequest>(&request.request)) {
        response.response = handle_leave_wall_request(*leave_request);
    }

    // Return the response
    return response;
}

void Defense::send_join_wall_request() {
    communication::JoinWallRequest join_request{};
    join_request.robot_id = robot_id_;
    communication::generate_uid(join_request);

    communication::PosAgentRequestWrapper communication_request{};
    communication_request.request = join_request;
    communication_request.target_agents = {};
    communication_request.urgent = false;
    communication_request.broadcast = true;

    communication_requests_.push_back(communication_request);

    current_state_ = WALLING;
}

void Defense::send_leave_wall_request() {
    communication::LeaveWallRequest leave_request{};
    leave_request.robot_id = robot_id_;
    communication::generate_uid(leave_request);

    communication::PosAgentRequestWrapper communication_request{};
    communication_request.request = leave_request;
    communication_request.target_agents = walling_robots_;
    communication_request.urgent = true;
    communication_request.broadcast = false;

    communication_requests_.push_back(communication_request);
}

communication::JoinWallResponse Defense::handle_join_wall_request(
    communication::JoinWallRequest join_request) {
    for (size_t i = 0; i < walling_robots_.size(); i++) {
        if (walling_robots_[i] == join_request.robot_id) {
            break;
        } else if (walling_robots_[i] > join_request.robot_id) {
            walling_robots_.insert(walling_robots_.begin() + i, join_request.robot_id);
            waller_id_ = get_waller_id();
            break;
        } else if (i == walling_robots_.size() - 1) {
            walling_robots_.push_back(join_request.robot_id);
            waller_id_ = get_waller_id();
        }
    }

    communication::JoinWallResponse join_response{};
    join_response.robot_id = robot_id_;
    communication::generate_uid(join_response);

    return join_response;
}

communication::Acknowledge Defense::handle_leave_wall_request(
    communication::LeaveWallRequest leave_request) {
    if (robot_id_ != leave_request.robot_id) {
        for (int i = walling_robots_.size() - 1; i > 0; i--) {
            if (walling_robots_[i] == leave_request.robot_id) {
                walling_robots_.erase(walling_robots_.begin() + i);
                waller_id_ = get_waller_id();
                break;
            } else if (walling_robots_[i] < leave_request.robot_id) {
                break;
            }
        }
    }

    communication::Acknowledge acknowledge_response{};
    communication::generate_uid(acknowledge_response);

    return acknowledge_response;
}

void Defense::handle_join_wall_response(communication::JoinWallResponse join_response) {
    for (size_t i = 0; i < walling_robots_.size(); i++) {
        if (walling_robots_[i] == join_response.robot_id) {
            return;
        } else if (walling_robots_[i] > join_response.robot_id) {
            walling_robots_.insert(walling_robots_.begin() + i, join_response.robot_id);
            waller_id_ = get_waller_id();
            return;
        } else if (i == walling_robots_.size() - 1) {
            walling_robots_.push_back(join_response.robot_id);
            waller_id_ = get_waller_id();
        }
    }
}

int Defense::get_waller_id() {
    return find(walling_robots_.begin(), walling_robots_.end(), robot_id_) -
           walling_robots_.begin() + 1;
}

void Defense::die() {
    if (current_state_ == WALLING) {
        send_leave_wall_request();
    }
}

void Defense::revive() { current_state_ = JOINING_WALL; }

rj_geometry::Point Defense::calculate_best_shot() const {
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
double Defense::distance_from_their_robots(rj_geometry::Point tail, rj_geometry::Point head) const {
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

bool Defense::ball_in_red() const {
    auto& ball_pos = this->last_world_state_->ball.position;
    return (field_dimensions_.our_defense_area().contains_point(ball_pos) ||
            field_dimensions_.their_defense_area().contains_point(ball_pos) ||
            !field_dimensions_.field_rect().contains_point(ball_pos));
}

bool Defense::we_in_red() const {
    auto& our_pos = this->last_world_state_->get_robot(true, robot_id_).pose.position();
    return (field_dimensions_.our_defense_area().contains_point(our_pos) ||
            field_dimensions_.their_defense_area().contains_point(our_pos) ||
            !field_dimensions_.field_rect().contains_point(our_pos));
}

bool Defense::we_are_closer_than_enemies() const {
    auto& their_robots = this->last_world_state_->their_robots;

    rj_geometry::Point my_robot_position = this->last_world_state_->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = this->last_world_state_->ball.position;
    double my_distance_to_ball = my_robot_position.dist_to(ball_position);

    for (auto enemy : their_robots) {
        if (enemy.pose.position().dist_to(ball_position) < my_distance_to_ball) {
            return false; // AN ENEMY IS CLOSER
        }
    }
    return true;
}

bool Defense::we_are_closer_than_teammates() const {
    auto& our_robots = this->last_world_state_->our_robots;

    rj_geometry::Point my_robot_position = this->last_world_state_->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = this->last_world_state_->ball.position;
    double my_distance_to_ball = my_robot_position.dist_to(ball_position);

    for (size_t i = 0; i < our_robots.size(); ++i) {
        if (i == robot_id_) { continue; } // don't count urself
        if (our_robots[i].pose.position().dist_to(ball_position) < my_distance_to_ball) {
            return false; // A TEAMMATE IS CLOSER
        }
    }
    return true;
}

}  // namespace strategy
