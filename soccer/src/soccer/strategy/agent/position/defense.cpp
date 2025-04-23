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

    bool we_are_closest;
    auto& our_robots = this->last_world_state_->our_robots;;
    auto& their_robots = this->last_world_state_->their_robots;;
    double min_dist;
    switch (current_state_) {
        case IDLING:
            break;
        case JOINING_WALL:
            send_join_wall_request();
            // SPDLOG_INFO("join wall {}", robot_id_);
            next_state = WALLING;
            walling_robots_ = {(u_int8_t)robot_id_};
            break;
        case WALLING:
            // If a wall is already full,
            // Remove the robot with the highest ID from a wall
            // and make them a marker instead.
            // if (walling_robots_.size() > kMaxWallers &&
            //     this->robot_id_ == *max_element(walling_robots_.begin(), walling_robots_.end())) {
            //     // send_leave_wall_request();
            //     // SPDLOG_INFO("leave wall {}", robot_id_);
            // }
            we_are_closest = true;
            our_robots = this->last_world_state_->our_robots;

            for (size_t i = 0; i < our_robots.size(); ++i) {
                if (i == robot_id_) {
                    continue;
                }

                rj_geometry::Point uspos = our_robots[i].pose.position();

                if (uspos.dist_to(ball_position) < distance_to_ball) {
                    we_are_closest = false;
                    break;
                }
            }


            if (we_are_closest) {
                their_robots = this->last_world_state_->their_robots;
                min_dist = 1000;
                for (auto enemy : their_robots) {
                    rj_geometry::Point enemypos = enemy.pose.position();

                    if (enemypos.dist_to(ball_position) < min_dist) {
                        min_dist = enemypos.dist_to(ball_position);
                    }
                }

                if (distance_to_ball < min_dist) {
                    next_state = WALLER_STEAL;
                }
            }
        

            break;
        case WALLER_STEAL:
            their_robots = this->last_world_state_->their_robots;
            min_dist = 1000;
            for (auto enemy : their_robots) {
                rj_geometry::Point enemypos = enemy.pose.position();

                if (enemypos.dist_to(ball_position) < min_dist) {
                    min_dist = enemypos.dist_to(ball_position);
                }
            }

            if (distance_to_ball >= min_dist) {
                next_state = JOINING_WALL;
            }
            if (check_is_done()) {
                target_ = calculate_best_shot();
                next_state = KICK;
            }
            break;
        case KICK:
            if (check_is_done()) {
                next_state = JOINING_WALL;
            }
            break;
            
    }

    return next_state;
}

std::optional<RobotIntent> Defense::state_to_task(RobotIntent intent) {
    if (current_state_ == IDLING) {
        auto empty_motion_cmd = planning::MotionCommand{};
        intent.motion_command = empty_motion_cmd;
        return intent;
        // DO NOTHING
    } else if (current_state_ == WALLING) {
        if (!walling_robots_.empty() && waller_id_ != -1) {
            Waller waller{waller_id_, walling_robots_};
            return waller.get_task(intent, last_world_state_, this->field_dimensions_);
        }
    } else if (current_state_ == WALLER_STEAL) {
        planning::LinearMotionInstant target{field_dimensions_.their_goal_loc()};
        auto pivot_cmd = planning::MotionCommand{"line_pivot", target, planning::FaceTarget{}, false, last_world_state_->ball.position};
        pivot_cmd.pivot_radius = kRobotRadius * 2.5;
        intent.motion_command = pivot_cmd;
        
        return intent;
    } else if (current_state_ == KICK) {
        auto line_kick_cmd = planning::MotionCommand{"line_kick", planning::LinearMotionInstant{target_}};
        intent.motion_command = line_kick_cmd;
        intent.shoot_mode = RobotIntent::ShootMode::KICK;
        intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
        intent.kick_speed = 4.0;
        
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

}  // namespace strategy
