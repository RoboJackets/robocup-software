#include "defense.hpp"

namespace strategy {

Defense::Defense(int r_id) : Position(r_id, "Defense") {}

Defense::~Defense() {
    die();
}

Defense::Defense(const Position& other) : Position{other} {
    position_name_ = "Defense";
    walling_robots_ = {};
}

std::optional<RobotIntent> Defense::derived_get_task(RobotIntent intent) {
    current_state_ = update_state();
    //waller_id_ = get_waller_id();
    return state_to_task(intent);
}

std::string Defense::get_current_state() {
    return std::string{"Defense"} + std::to_string(static_cast<int>(current_state_));
}

Defense::State Defense::update_state() {
    rj_geometry::Point robot_position = last_world_state_->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = last_world_state_->ball.position;
    double distance_to_ball = robot_position.dist_to(ball_position);

    if (current_state_ != WALLING && current_state_ != JOINING_WALL && waller_id_ != -1) {
        send_leave_wall_request();
        walling_robots_ = {(u_int8_t)robot_id_};
        waller_id_ = -1;
        return DEFAULT;
    }

    switch (current_state_) {
        case DEFAULT: {
            return JOINING_WALL;
        }
        case JOINING_WALL: {
            send_join_wall_request(); // sets waller_id_
            SPDLOG_INFO("{} joining wall at wall pos {}", robot_id_, waller_id_);
            next_state = WALLING;
            walling_robots_ = {(u_int8_t)robot_id_};
            break;
        }
        case WALLING: {
            break;
        }
    }

    return next_state;
}

std::optional<RobotIntent> Defense::state_to_task(RobotIntent intent) {
    cached_ball_pos_ = get_ball_pos();

    if (current_state_ == DEFAULT) {
        return std::nullopt;
    } else if (current_state_ == JOINING_WALL) {
        return std::nullopt;
    } else if (current_state_ == WALLING) {
        if (!walling_robots_.empty() && waller_id_ != -1) {
            Waller waller{waller_id_, walling_robots_};
            // return waller.get_task(intent, last_world_state_, this->field_dimensions_);
            return waller.get_task_with_ball(intent, last_world_state_, this->field_dimensions_, cached_ball_pos_);
        }
    }

    return std::nullopt;
}

bool Defense::is_alive(u_int8_t concerned_id) {
    if (!last_world_state_->get_robot(concerned_id, true).visible) {
        return false;
    } else /**if (another condition) {} else*/ {
        return true;
    }
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

void Defense::derived_acknowledge_pass() {}

void Defense::derived_pass_ball() {}

void Defense::derived_acknowledge_ball_in_transit() {}

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

rj_geometry::Point Defense::get_ball_pos() const {
    if (last_world_state_->ball.visible) {
        return last_world_state_->ball.position;
    } else {
        return cached_ball_pos_;
    }
}

}  // namespace strategy
