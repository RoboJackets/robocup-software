#include "defense.hpp"

namespace strategy {

Defense::Defense(int r_id) : Position(r_id, "Defense"), marker_{field_dimensions_} {}

Defense::Defense(const Position& other) : Position{other}, marker_{field_dimensions_} {
    position_name_ = "Defense";
    walling_robots_ = {};
    marking_robots_ = {};
    marked_robots_ = {};
    kMaxMarkers = 3;
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
    State next_state = current_state_;
    // handle transitions between states
    WorldState* world_state = last_world_state_;

    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = world_state->ball.position;
    double distance_to_ball = robot_position.dist_to(ball_position);

    if (current_state_ != WALLING && current_state_ != JOINING_WALL && waller_id_ != -1) {
        // SPDLOG_INFO("WE ARE LEAVING THE WALL");
        send_leave_wall_request();
        walling_robots_ = {(u_int8_t)robot_id_};
        waller_id_ = -1;
    }

    if (current_state_ != MARKING && current_state_ != ENTERING_MARKING && marker_id_ != -1) {
        send_leave_mark_request(marker_.get_target());
        marker_id_ = -1;
    }

    switch (current_state_) {
        case IDLING:
            break;
        case JOINING_WALL:
            send_join_wall_request();
            SPDLOG_INFO("join wall {}", robot_id_);
            next_state = WALLING;
            walling_robots_ = {(u_int8_t)robot_id_};
            break;
        case WALLING: {
            // If a wall is already full,
            // Remove the robot with the highest ID from a wall
            // and make them a marker instead.
            // if (walling_robots_.size() > kMaxWallers &&
            //     this->robot_id_ == *max_element(walling_robots_.begin(), walling_robots_.end())) {

            //     SPDLOG_INFO("back again");
            //     int deepest_robot = 0;
            //     for (int i = 0; i < kNumShells; i++) {
            //         // if (marked_robots_.find(i) != marked_robots_.end()) {
            //         //     continue;
            //         // }
                    // if (world_state->get_robot(false, i).pose.position().y() < world_state->get_robot(false, deepest_robot).pose.position().y() && world_state->get_robot(false, i).pose.position().y() > 0
                    // && world_state->get_robot(false, i).pose.position().x() > -3 && world_state->get_robot(false, i).pose.position().x() < 3) {
                    //     deepest_robot = i;
                    // }
            //     }

            //     if (world_state->get_robot(false, deepest_robot).pose.position().y() < marking_y_bound) {
            //         SPDLOG_INFO("Target set to {}", deepest_robot);
            //         send_leave_wall_request();
            //         next_state = ENTERING_MARKING;
            //         marker_.set_target(deepest_robot);
            //     }
            // }

            // SPDLOG_INFO("Robot {} is marking", robot_id_);

            // For when I get communication working
            int deepest_robot = 0;
            for (int i = 0; i < kNumShells; i++) {
                if (std::find(marked_robots_.begin(), marked_robots_.end(), i) != marked_robots_.end()) {
                    continue;
                }
                if (world_state->get_robot(false, i).pose.position().y() < world_state->get_robot(false, deepest_robot).pose.position().y() && world_state->get_robot(false, i).pose.position().y() > 0
                && world_state->get_robot(false, i).pose.position().x() > -3 && world_state->get_robot(false, i).pose.position().x() < 3) {
                    deepest_robot = i;
                }
            }

            // SPDLOG_INFO("Deepest robot is {}", deepest_robot);
            // SPDLOG_INFO("Marking robots is less than max markers: {}", marking_robots_.size());
            // SPDLOG_INFO("Walling robots is at least max: {}", walling_robots_.size() >= kMaxWallers);

            if (walling_robots_.size() >= kMaxWallers && marking_robots_.size() < kMaxMarkers && world_state->get_robot(false, deepest_robot).pose.position().y() < marking_y_bound) {
                SPDLOG_INFO("Wallers currently: {}", walling_robots_.size());
                SPDLOG_INFO("Marking robot size: {}", marking_robots_.size());
                // SPDLOG_INFO("Max markers: {}", kMaxMarkers);
                
                marker_.set_target(deepest_robot);
                marked_robots_.push_back(deepest_robot);
                marking_robots_.push_back(robot_id_);

                SPDLOG_INFO("I am currently marking {}", marker_.get_target());

                send_leave_wall_request();

                SPDLOG_INFO("I am {}", robot_id_);
                for (int i = 0; i < walling_robots_.size(); i++) {
                    SPDLOG_INFO("Robot {} is walling as of now", walling_robots_[i]);
                }

                next_state = ENTERING_MARKING;
            }


            break;
        }
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
        case MARKING:
            // if (marker_.get_target() == -1 || marker_.target_out_of_bounds(world_state)) {
            if (marker_.target_out_of_bounds(world_state)) {
                next_state = JOINING_WALL;
                // SPDLOG_INFO("Number of markers currently is {}", marking_robots_.size());
                // marking_robots_.erase(std::remove(marking_robots_.begin(), marking_robots_.end(), robot_id_), marking_robots_.end());
            }
            break;
        case ENTERING_MARKING:
            // marker_.choose_target(world_state);
            SPDLOG_INFO("Robot {} is entering marking", robot_id_);

            // for (int i = 0; i < marked_robots_.size(); i++) {
            //     SPDLOG_INFO("Robot {} is marking {}", marking_robots_[i], marked_robots_[i]);
            // }

            // for (int i = 0; i < walling_robots_.size(); i++) {
            //     SPDLOG_INFO("Robot {} is walling as of now", walling_robots_[i]);
            // }
            
            int target_id = marker_.get_target();
            if (target_id == -1) {
                next_state = ENTERING_MARKING;
            } else {
                send_join_mark_request(target_id);
                next_state = MARKING;
            }
    }

    // SPDLOG_INFO("Size of marking robots: {}", marking_robots_.size());
    // if (marker_.get_target() != -1) {
    //     SPDLOG_INFO("Target: {}", marker_.get_target());
    // }
    // if (marking_robots_.size() > 0) {
    //     SPDLOG_INFO("Marking robot: {}", marking_robots_[0]);
    // }

    // if (robot_id_ == 5) {
    //     SPDLOG_INFO("My state is {}", current_state_);
    // }
    

    return next_state;
}

std::optional<RobotIntent> Defense::state_to_task(RobotIntent intent) {
    // if (robot_id_ == 2) {
    //     SPDLOG_INFO("{} current state of 2", current_state_);
    // }
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
        if (!walling_robots_.empty() && waller_id_ != -1) {
            Waller waller{waller_id_, walling_robots_};
            return waller.get_task(intent, last_world_state_, this->field_dimensions_);
        }
    } else if (current_state_ == FACING) {
        rj_geometry::Point robot_position =
            last_world_state_->get_robot(true, robot_id_).pose.position();
        auto current_location_instant =
            planning::LinearMotionInstant{robot_position, rj_geometry::Point{0.0, 0.0}};
        auto face_ball = planning::FaceBall{};
        auto face_ball_cmd =
            planning::MotionCommand{"path_target", current_location_instant, face_ball};
        intent.motion_command = face_ball_cmd;
        return intent;
    } else if (current_state_ == ENTERING_MARKING) {
        // Prepares a robot for marking. NOTE: May update to add move to center of field
        auto empty_motion_cmd = planning::MotionCommand{};
        intent.motion_command = empty_motion_cmd;
        return intent;
    } else if (current_state_ == MARKING) {
        // Marker marker = Marker((u_int8_t) robot_id_);
        return marker_.get_task(intent, last_world_state_, this->field_dimensions_);
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
    
    SPDLOG_INFO("Received a communication request of type: {}", request.request.index());
    
    // Call to super
    communication::PosAgentResponseWrapper response =
        Position::receive_communication_request(request);

    // SPDLOG_INFO("Received a communication request of type: {}", request.request.index());

    // Handle join and leave wall request
    if (const communication::JoinWallRequest* join_request =
            std::get_if<communication::JoinWallRequest>(&request.request)) {
        response.response = handle_join_wall_request(*join_request);
    } else if (const communication::LeaveWallRequest* leave_request =
                   std::get_if<communication::LeaveWallRequest>(&request.request)) {
        // SPDLOG_INFO("Step 1 of leaving");
        response.response = handle_leave_wall_request(*leave_request);
    } 

    if (const communication::JoinMarkingRequest* join_mark_request = 
                    std::get_if<communication::JoinMarkingRequest>(&request.request)) {
        SPDLOG_INFO("We are here");
        response.response = handle_join_marking_request(*join_mark_request);
    } else if (const communication::LeaveMarkingRequest* leave_mark_request =
                    std::get_if<communication::LeaveMarkingRequest>(&request.request)) {
        SPDLOG_INFO("How?");
        response.response = handle_leave_marking_request(*leave_mark_request);
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
    SPDLOG_INFO("Communication request size is (in walling) {}", communication_requests_.size());

    current_state_ = WALLING;
}

void Defense::send_join_mark_request(int mark_id) {

    // SPDLOG_INFO("Step 1");

    communication::JoinMarkingRequest join_request{};
    join_request.robot_id = robot_id_;
    join_request.marked_robot_id = mark_id;
    communication::generate_uid(join_request);

    communication::PosAgentRequestWrapper communication_request{};
    communication_request.request = join_request;
    communication_request.target_agents = {};
    communication_request.urgent = false;
    communication_request.broadcast = true;

    SPDLOG_INFO("Communication request type: {}",communication_request.request.index());
    communication_requests_.push_back(communication_request);
    SPDLOG_INFO("Communication request size is {}", communication_requests_.size());
    SPDLOG_INFO("Front of communication requests is of type {}", communication_requests_[0].request.index());
    SPDLOG_INFO("Communication request type confirmation: {}",communication_requests_.back().request.index());

    current_state_ = MARKING;
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

void Defense::send_leave_mark_request(int mark_id) {
    communication::LeaveMarkingRequest leave_request{};
    leave_request.robot_id = robot_id_;
    leave_request.marked_robot_id = mark_id;
    communication::generate_uid(leave_request);

    communication::PosAgentRequestWrapper communication_request{};
    communication_request.request = leave_request;
    communication_request.target_agents = marking_robots_;
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

communication::JoinMarkingResponse Defense::handle_join_marking_request(
    communication::JoinMarkingRequest join_request) {
    SPDLOG_INFO("Handling join marking request for robot {}", join_request.robot_id);

    // If the list is empty, simply add the robot
    if (marking_robots_.empty()) {
        marking_robots_.push_back(join_request.robot_id);
        marked_robots_.push_back(join_request.marked_robot_id);
    } else {
        // Ensure robots are stored in ascending order
        auto it = std::lower_bound(marking_robots_.begin(), marking_robots_.end(), join_request.robot_id);
        size_t index = std::distance(marking_robots_.begin(), it);
        
        // Insert at correct position
        marking_robots_.insert(it, join_request.robot_id);
        marked_robots_.insert(marked_robots_.begin() + index, join_request.marked_robot_id);
    }

    marker_id_ = get_marker_id();
    
    SPDLOG_INFO("Updated marking robots list: {}", fmt::join(marking_robots_, ", "));
    SPDLOG_INFO("Updated marked robots list: {}", fmt::join(marked_robots_, ", "));

    communication::JoinMarkingResponse join_response{};
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

communication::Acknowledge Defense::handle_leave_marking_request(
    communication::LeaveMarkingRequest leave_request) {
    if (robot_id_ != leave_request.robot_id) {
        for (int i = marking_robots_.size() - 1; i > 0; i--) {
            if (marking_robots_[i] == leave_request.robot_id) {
                marking_robots_.erase(marking_robots_.begin() + i);
                marked_robots_.erase(marked_robots_.begin() + i);
                marker_id_ = get_marker_id();
                break;
            } else if (marking_robots_[i] < leave_request.robot_id) {
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

int Defense::get_marker_id() {
    return find(marking_robots_.begin(), marking_robots_.end(), robot_id_) - marking_robots_.begin() + 1;
}

void Defense::die() {
    if (current_state_ == WALLING) {
        send_leave_wall_request();
    }
}

void Defense::revive() { current_state_ = JOINING_WALL; }

}  // namespace strategy
