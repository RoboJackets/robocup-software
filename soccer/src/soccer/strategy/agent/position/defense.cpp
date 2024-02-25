#include "defense.hpp"

namespace strategy {

Defense::Defense(int r_id) : Position(r_id) { position_name_ = "Defense"; }

std::optional<RobotIntent> Defense::derived_get_task(RobotIntent intent) {
    current_state_ = update_state();
    return state_to_task(intent);
}

Defense::State Defense::update_state() {
    State next_state = current_state_;
    // handle transitions between states
    WorldState* world_state = last_world_state_;

    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = world_state->ball.position;
    double distance_to_ball = robot_position.dist_to(ball_position);

    if (current_state_ != WALLING && current_state_ != JOINING_WALL && waller_id_ != -1) {
        send_leave_wall_request();
        walling_robots_ = {(u_int8_t)robot_id_};
        waller_id_ = -1;
    }

    switch (current_state_) {
        case IDLING:
            break;
        case JOINING_WALL:
            send_join_wall_request();
            next_state = WALLING;
            walling_robots_ = {(u_int8_t)robot_id_};
            break;
        case WALLING:
            break;
        case SEARCHING:
            break;
        case RECEIVING:
            // transition to idling if we are close enough to the ball
            if (distance_to_ball < ball_receive_distance_) {
                next_state = IDLING;
            }
            break;
        case PASSING:
            // transition to idling if we no longer have the ball (i.e. it was passed or it was
            // stolen)
            if (check_is_done()) {
                next_state = IDLING;
            }

            if (distance_to_ball > ball_lost_distance_) {
                next_state = IDLING;
            }
            break;
        case FACING:
            if (check_is_done()) {
                next_state = IDLING;
            }
    }

    return next_state;
}

std::optional<RobotIntent> Defense::state_to_task(RobotIntent intent) {
    if (current_state_ == IDLING) {
        auto empty_motion_cmd = planning::MotionCommand{};
        intent.motion_command = empty_motion_cmd;
        return intent;
        // DO NOTHING
    } else if (current_state_ == SEARCHING) {
        // TODO(https://app.clickup.com/t/8677qektb): Define defensive searching behavior
    } else if (current_state_ == RECEIVING) {
        // check how far we are from the ball
        // TODO(https://app.clickup.com/t/8677rrgjn): Convert RECEIVING state into role_interface
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
            // intercept the ball
            chasing_ball = true;
            auto collect_cmd = planning::MotionCommand{"collect"};
            intent.motion_command = collect_cmd;
        }
        return intent;
    } else if (current_state_ == PASSING) {
        // attempt to pass the ball to the target robot
        rj_geometry::Point target_robot_pos =
            last_world_state_->get_robot(true, target_robot_id).pose.position();
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
    } else if (current_state_ == WALLING) {
        if (!walling_robots_.empty()) {
            Waller waller{waller_id_, (int)walling_robots_.size()};
            return waller.get_task(intent, last_world_state_, this->field_dimensions_);
        }
    } else if (current_state_ = FACING) {
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
    for (int i = 0; i < walling_robots_.size(); i++) {
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
    for (int i = walling_robots_.size() - 1; i > 0; i--) {
        if (walling_robots_[i] == leave_request.robot_id) {
            walling_robots_.erase(walling_robots_.begin() + i);
            waller_id_ = get_waller_id();
            break;
        } else if (walling_robots_[i] < leave_request.robot_id) {
            break;
        }
    }

    communication::Acknowledge acknowledge_response{};
    communication::generate_uid(acknowledge_response);

    return acknowledge_response;
}

void Defense::handle_join_wall_response(communication::JoinWallResponse join_response) {
    for (int i = 0; i < walling_robots_.size(); i++) {
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

void Defense::derived_acknowledge_pass() { current_state_ = FACING; }

void Defense::derived_pass_ball() { current_state_ = PASSING; }

void Defense::derived_acknowledge_ball_in_transit() {
    current_state_ = RECEIVING;
    chasing_ball = false;
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

}  // namespace strategy
