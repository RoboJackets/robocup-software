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
    WorldState* world_state = this->world_state();

    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = world_state->ball.position;
    double distance_to_ball = robot_position.dist_to(ball_position);

    switch (current_state_) {
        case IDLING:
            break;
        case JOINING_WALL:
            send_join_wall_request();
            next_state = WALLING;
            walling_robots = {(u_int8_t) robot_id_};
            break;
        case WALLING:
            break;
        case LEAVING_WALL:
            send_leave_wall_request();
            next_state = IDLING;
            walling_robots = {(u_int8_t) robot_id_};
            waller_id = -1;
            break;
        case SEARCHING:
            break;
        case RECEIVING:
            // transition to idling if we are close enough to the ball
            if (distance_to_ball < BALL_RECEIVE_DISTANCE) {
                next_state = IDLING;
            }
            break;
        case PASSING:
            // transition to idling if we no longer have the ball (i.e. it was passed or it was
            // stolen)
            if (distance_to_ball > BALL_LOST_DISTANCE) {
                next_state = IDLING;
            }
            break;
    }

    return next_state;
}

std::optional<RobotIntent> Defense::state_to_task(RobotIntent intent) {
    if (current_state_ == IDLING) {
        // DO NOTHING
    } else if (current_state_ == SEARCHING) {
        // TODO: Define defensive searching behavior
    } else if (current_state_ == RECEIVING) {
        // check how far we are from the ball
        rj_geometry::Point robot_position =
            world_state()->get_robot(true, robot_id_).pose.position();
        rj_geometry::Point ball_position = world_state()->ball.position;
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
            SPDLOG_INFO("\033[92mrobot {} settling the ball\033[0m", robot_id_);
            auto collect_cmd = planning::MotionCommand{"collect"};
            intent.motion_command = collect_cmd;
        }
        return intent;
    } else if (current_state_ == PASSING) {
        // attempt to pass the ball to the target robot
        SPDLOG_INFO("\033[92mrobot {} passing ball\033[0m", robot_id_);
        rj_geometry::Point target_robot_pos =
            world_state()->get_robot(true, target_robot_id).pose.position();
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
        if (walling_robots.size() > 1) {
            Waller waller{waller_id};
            return waller.get_task(intent, world_state());
        }
    }

    return std::nullopt;
}

communication::Acknowledge Defense::acknowledge_pass(
    communication::IncomingPassRequest incoming_pass_request) {
    // Call to super
    communication::Acknowledge acknowledge_response =
        Position::acknowledge_pass(incoming_pass_request);
    // Update current state
    current_state_ = FACING;
    // Return acknowledge response
    return acknowledge_response;
}

void Defense::pass_ball(int robot_id) {
    // Call to super
    Position::pass_ball(robot_id);
    // Update current state
    current_state_ = PASSING;
}

communication::Acknowledge Defense::acknowledge_ball_in_transit(
    communication::BallInTransitRequest ball_in_transit_request) {
    // Call to super
    communication::Acknowledge acknowledge_response =
        Position::acknowledge_ball_in_transit(ball_in_transit_request);
    // Update current state
    current_state_ = RECEIVING;
    // Return acknowledge response
    return acknowledge_response;
}

void Defense::receive_communication_response(communication::AgentPosResponseWrapper response) {
    // Call to super
    Position::receive_communication_response(response);

    // Handle join wall response
    if (const communication::JoinWallRequest* join_request = std::get_if<communication::JoinWallRequest>(&response.associated_request)) {
        for (u_int32_t i = 0; i < response.responses.size(); i++) {
            if (const communication::JoinWallResponse* join_response = std::get_if<communication::JoinWallResponse>(&response.responses[i])) {
                handle_join_wall_response(*join_response);
            }
        }
    }
}

communication::PosAgentResponseWrapper Defense::receive_communication_request(communication::AgentPosRequestWrapper request) {
    // Call to super
    communication::PosAgentResponseWrapper response = Position::receive_communication_request(request);

    // Handle join and leave wall request
    if (const communication::JoinWallRequest* join_request = std::get_if<communication::JoinWallRequest>(&request.request)) {
        response.response = handle_join_wall_request(*join_request);
    } else if (const communication::LeaveWallRequest* leave_request = std::get_if<communication::LeaveWallRequest>(&request.request)) {
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

    communication_request_ = communication_request;

    current_state_ = WALLING;
}

void Defense::send_leave_wall_request() {
    communication::LeaveWallRequest leave_request{};
    leave_request.robot_id = robot_id_;
    communication::generate_uid(leave_request);

    communication::PosAgentRequestWrapper communication_request{};
    communication_request.request = leave_request;
    communication_request.target_agents = walling_robots;
    communication_request.urgent = true;
    communication_request.broadcast = false;

    communication_request_ = communication_request;
}

communication::JoinWallResponse Defense::handle_join_wall_request(communication::JoinWallRequest join_request) {
    for (int i = 0; i < walling_robots.size(); i++) {
        if (walling_robots[i] == join_request.robot_id) {
            break;
        } else if (walling_robots[i] > join_request.robot_id) {
            walling_robots.insert(walling_robots.begin() + i, join_request.robot_id);
            waller_id = find(walling_robots.begin(), walling_robots.end(), robot_id_) - walling_robots.begin();
            break;
        }
    }

    communication::JoinWallResponse join_response{};
    join_response.robot_id = robot_id_;
    communication::generate_uid(join_response);

    return join_response;
}

communication::Acknowledge Defense::handle_leave_wall_request(communication::LeaveWallRequest leave_request) {
    for (int i = walling_robots.size() - 1; i > 0; i--) {
        if (walling_robots[i] == leave_request.robot_id) {
            walling_robots.erase(walling_robots.begin() + i);
            waller_id = find(walling_robots.begin(), walling_robots.end(), robot_id_) - walling_robots.begin();
            break;
        } else if (walling_robots[i] > leave_request.robot_id) {
            break;
        }
    }

    communication::Acknowledge acknowledge_response{};
    communication::generate_uid(acknowledge_response);

    return acknowledge_response;
}

void Defense::handle_join_wall_response(communication::JoinWallResponse join_response) {
    for (int i = 0; i < walling_robots.size(); i++) {
        if (walling_robots[i] == join_response.robot_id) {
            return;
        } else if (walling_robots[i] > join_response.robot_id) {
            walling_robots.insert(walling_robots.begin() + i, join_response.robot_id);
            waller_id = find(walling_robots.begin(), walling_robots.end(), robot_id_) - walling_robots.begin();
            return;
        }
    }
}

}  // namespace strategy
