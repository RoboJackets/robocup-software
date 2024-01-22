#include "offense.hpp"

namespace strategy {

Offense::Offense(int r_id) : Position(r_id) {
    position_name_ = "Offense";
    current_state_ = IDLING;
}

std::optional<RobotIntent> Offense::derived_get_task(RobotIntent intent) {
    // SPDLOG_INFO("MY ID: {} in offense derived task!\n", robot_id_);
    current_state_ = update_state();
    // SPDLOG_INFO("My current offense state is {}", current_state_);
    return state_to_task(intent);
}

Offense::State Offense::update_state() {
    State next_state = current_state_;
    // handle transitions between current state
    WorldState* world_state = this->last_world_state_;

    // if no ball found, stop and return to box immediately
    if (!world_state->ball.visible) {
        return current_state_;
    }

    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = world_state->ball.position;
    double distance_to_ball = robot_position.dist_to(ball_position);

    if (current_state_ == IDLING) {
        // send_scorer_request();
        if (robot_id_ == 4) {
            next_state = AWAITING_SEND_PASS;
        } else {
            next_state = SEEKING;
        }
    } else if (current_state_ == SEARCHING) {
        if (scorer_) {
            next_state = STEALING;
        }
    } else if (current_state_ == PASSING) {
        // transition to idling if we no longer have the ball (i.e. it was passed or it was
        // stolen)
        if (check_is_done()) {
            next_state = IDLING;
        }

        if (distance_to_ball > ball_lost_distance_) {
            next_state = IDLING;
        }
    } else if (current_state_ == PREPARING_SHOT) {
        if (check_is_done()) {
            next_state = SHOOTING;
        }
    } else if (current_state_ == SHOOTING) {
        // transition to idling if we no longer have the ball (i.e. it was passed or it was
        // stolen)
        if (check_is_done() || distance_to_ball > ball_lost_distance_) {
            send_reset_scorer_request();
            next_state = SEARCHING;
        }
    } else if (current_state_ == RECEIVING) {
        // transition to idling if we are close enough to the ball
        if (distance_to_ball < 2*ball_receive_distance_) {
            next_state = IDLING;
        }
    } else if (current_state_ == STEALING) {
        // The collect planner check_is_done() is wonky so I added a second clause to check
        // distance
        if (check_is_done() || distance_to_ball < ball_receive_distance_) {
            // send direct pass request to robot 4
            if (scorer_) {
                next_state = PREPARING_SHOT;
            } else {
                /* send_direct_pass_request({4}); */
                /* next_state = SEARCHING; */
            }
        }
    } else if (current_state_ == FACING) {
        if (check_is_done()) {
            next_state = IDLING;
        }
    } else if (current_state_ == AWAITING_SEND_PASS) {
        auto cur = std::chrono::high_resolution_clock::now();
        auto diff = cur - start;
        if (distance_to_ball < ball_lost_distance_ && diff.count() > 0.2){
            Position::broadcast_direct_pass_request();
            start = std::chrono::high_resolution_clock::now();
        }
    } else if (current_state_ ==  SEEKING) {
        // if the ball comes close to it while it's trying to seek, it should no longer be trying to seek
        if (distance_to_ball < ball_receive_distance_) {
            // next_state = IDLING;
        }
    }
    

    return next_state;
}

std::optional<RobotIntent> Offense::state_to_task(RobotIntent intent) {
    if (current_state_ == IDLING) {
        // Do nothing
        auto empty_motion_cmd = planning::MotionCommand{};
        intent.motion_command = empty_motion_cmd;
        return intent;
    } else if (current_state_ == SEARCHING) {
        // DEFINE SEARCHING BEHAVIOR
        auto empty_motion_cmd = planning::MotionCommand{};
        intent.motion_command = empty_motion_cmd;
        return intent;
    } else if (current_state_ == PASSING) {
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
    } else if (current_state_ == PREPARING_SHOT) {
        // pivot around ball...
        auto ball_pt = last_world_state_->ball.position;

        // ...to face their goal
        rj_geometry::Point their_goal_pos = field_dimensions_.their_goal_loc();
        planning::LinearMotionInstant target_instant{their_goal_pos};

        auto pivot_cmd = planning::MotionCommand{"pivot"};
        pivot_cmd.target = target_instant;
        pivot_cmd.pivot_point = ball_pt;
        intent.motion_command = pivot_cmd;
        intent.dribbler_speed = 255.0;
        return intent;
    } else if (current_state_ == SHOOTING) {
        rj_geometry::Point their_goal_pos = field_dimensions_.their_goal_loc();
        planning::LinearMotionInstant target{their_goal_pos};
        auto line_kick_cmd = planning::MotionCommand{"line_kick", target};
        intent.motion_command = line_kick_cmd;
        intent.shoot_mode = RobotIntent::ShootMode::KICK;
        intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
        intent.kick_speed = 4.0;
        intent.is_active = true;
        return intent;
    } else if (current_state_ == RECEIVING) {
        // check how far we are from the ball
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
            // intercept the bal
            chasing_ball = true;
            auto collect_cmd = planning::MotionCommand{"collect"};
            intent.motion_command = collect_cmd;
        }
        return intent;
    } else if (current_state_ == STEALING) {
        // intercept the ball
        // if ball fast, use settle, otherwise collect
        if (last_world_state_->ball.velocity.mag() > 0.75) {
            auto settle_cmd = planning::MotionCommand{"settle"};
            intent.motion_command = settle_cmd;
            intent.dribbler_speed = 255.0;
            return intent;
        } else {
            auto collect_cmd = planning::MotionCommand{"collect"};
            intent.motion_command = collect_cmd;
            intent.dribbler_speed = 255.0;
            return intent;
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
    } else if (current_state_ == AWAITING_SEND_PASS) {
        auto empty_motion_cmd = planning::MotionCommand{};
        intent.motion_command = empty_motion_cmd;
        return intent;
    } else if (current_state_ == SEEKING) {
        //Determine target position for seeking
        rj_geometry::Point current_loc = last_world_state_->get_robot(true, robot_id_).pose.position(); 
        rj_geometry::Point curr_target = intent.motion_command.target.position;
        //Only look for a new position if we are done with current movement
        if (check_is_done() || target_pt == rj_geometry::Point{0, 0}) {
            target_pt = get_open_point(last_world_state_, current_loc, field_dimensions_);
        }
        planning::PathTargetFaceOption face_option = planning::FaceBall{};
        bool ignore_ball = false;
        planning::LinearMotionInstant goal{target_pt, rj_geometry::Point{0.0, 0.0}};
        intent.motion_command = planning::MotionCommand{"path_target", goal, face_option, ignore_ball};
        return intent;
    }

    // should be impossible to reach, but this is an EmptyMotionCommand
    return std::nullopt;
}

rj_geometry::Point Offense::get_open_point(const WorldState* world_state,
                                          rj_geometry::Point current_loc,
                                          FieldDimensions field_dimensions) {
    return Offense::calculate_open_point(3.0, .2, current_loc, world_state, field_dimensions);
}

rj_geometry::Point Offense::calculate_open_point(double current_prec, double min_prec,
                                                rj_geometry::Point current_point,
                                                const WorldState* world_state,
                                                FieldDimensions field_dimensions) {
    while (current_prec > min_prec) {
        rj_geometry::Point ball_pos = world_state->ball.position;
        rj_geometry::Point min = current_point;
        double min_val = max_los(ball_pos, current_point, world_state);
        double curr_val{};
        // Points in a current_prec radius of the current point, at 45 degree intervals
        std::vector<rj_geometry::Point> check_points{
            correct_point(current_point + rj_geometry::Point{current_prec, 0}, field_dimensions),
            correct_point(current_point + rj_geometry::Point{-current_prec, 0}, field_dimensions),
            correct_point(current_point + rj_geometry::Point{0, current_prec}, field_dimensions),
            correct_point(current_point + rj_geometry::Point{0, -current_prec}, field_dimensions),
            correct_point(
                current_point + rj_geometry::Point{current_prec * 0.707, current_prec * 0.707},
                field_dimensions),
            correct_point(
                current_point + rj_geometry::Point{current_prec * 0.707, -current_prec * 0.707},
                field_dimensions),
            correct_point(
                current_point + rj_geometry::Point{-current_prec * 0.707, current_prec * 0.707},
                field_dimensions),
            correct_point(
                current_point + rj_geometry::Point{-current_prec * 0.707, -current_prec * 0.707},
                field_dimensions)};

        for (auto point : check_points) {
            curr_val = max_los(ball_pos, point, world_state);
            if (curr_val < min_val) {
                min_val = curr_val;
                min = point;
            }
        }
        current_prec *= 0.5;
        current_point = min;
    }
    return current_point;
}

rj_geometry::Point Offense::random_noise(double prec) {
    double x = (double)rand() / RAND_MAX * prec;
    double y = (double)rand() / RAND_MAX * prec;
    return rj_geometry::Point{x, y};
}

rj_geometry::Point Offense::correct_point(rj_geometry::Point p, FieldDimensions field_dimensions) {
    double BORDER_BUFFER = .2;
    double x = p.x();
    double y = p.y();

    // X Border
    if (p.x() > field_dimensions.field_x_right_coord() - BORDER_BUFFER) {
        x = field_dimensions.field_x_right_coord() - BORDER_BUFFER;
    } else if (p.x() < field_dimensions.field_x_left_coord() + BORDER_BUFFER) {
        x = field_dimensions.field_x_left_coord() + BORDER_BUFFER;
    }

    // Y Border
    if (p.y() > field_dimensions.their_goal_loc().y() - BORDER_BUFFER) {
        y = field_dimensions.their_goal_loc().y() - BORDER_BUFFER;
    } else if (p.y() < field_dimensions.our_goal_loc().y() + BORDER_BUFFER) {
        y = field_dimensions.our_goal_loc().y() + BORDER_BUFFER;
    }

    // Goalie Boxes
    if ((y < 1.2 || y > 7.8) && fabs(x) < 1.2) {
        if (y > 4.5) {
            y = 8.0 - BORDER_BUFFER;
        } else {
            y = 1.0 + BORDER_BUFFER;
        }

        if (x > .5) {
            x = 1.0 + BORDER_BUFFER;
        } else {
            x = -1.0 - BORDER_BUFFER;
        }
    }

    // Assigns robots to horizontal thirds
    if (robot_id_ == 1) {
        // Assign left
        if (x > field_dimensions.field_x_left_coord() + field_dimensions.width() / 2) {
            x = field_dimensions.field_x_left_coord() + field_dimensions.width() / 2 -
                BORDER_BUFFER;
        }
    } else if (robot_id_ == 2) {
        // Assign right
        if (x < field_dimensions.field_x_right_coord() - field_dimensions.width() / 2) {
            x = field_dimensions.field_x_right_coord() - field_dimensions.width() / 2 +
                BORDER_BUFFER;
        }
    } else {
        // Assign middle
        if (x < field_dimensions.field_x_left_coord() + field_dimensions.width() / 3) {
            x = field_dimensions.field_x_left_coord() + field_dimensions.width() / 3 +
                BORDER_BUFFER;
        } else if (x > field_dimensions.field_x_right_coord() - field_dimensions.width() / 3) {
            x = field_dimensions.field_x_right_coord() - field_dimensions.width() / 3 -
                BORDER_BUFFER;
        }
    }

    return rj_geometry::Point(x, y);
}

double Offense::max_los(rj_geometry::Point ball_pos, rj_geometry::Point current_point,
                       const WorldState* world_state) {
    //Determines 'how good' a point is
    //A higher value is a worse point

    //Does not go into the goalie boxes
    rj_geometry::Rect goal_box{rj_geometry::Point{1, 8}, rj_geometry::Point{-1, 9}};
    if (goal_box.contains_point(current_point)) {
        return 10000000;
    }

    //Line of Sight Heuristic
    double max = 0;
    double curr_dp;
    for (auto robot : world_state->their_robots) {
        curr_dp = (current_point).norm().dot((robot.pose.position() - ball_pos).norm());
        curr_dp *= curr_dp;
        if (curr_dp > max) {
            max = curr_dp;
        }
    }

    //Whether the path from ball to the point is blocked
    //Same logic in passing to check if target is open
    rj_geometry::Segment pass_path{ball_pos, current_point};
    double min_robot_dist = 10000;
    float min_path_dist = 10000;
    for (auto bot : world_state->their_robots) {
        rj_geometry::Point opp_pos = bot.pose.position();
        min_robot_dist = std::min(min_robot_dist, current_point.dist_to(opp_pos));
        min_path_dist = std::min(min_path_dist, pass_path.dist_to(opp_pos));
    }

    for (auto bot : world_state->our_robots) {
        rj_geometry::Point ally_pos = bot.pose.position();
        min_robot_dist = std::min(min_robot_dist, current_point.dist_to(ally_pos));
        min_path_dist = std::min(min_path_dist, pass_path.dist_to(ally_pos));
    }
    
    min_path_dist = 0.1 / min_path_dist;
    min_robot_dist = 0.1 / min_robot_dist;

    //More Line of Sight Heuristics
    for (auto robot : world_state->our_robots) {
        curr_dp = (current_point - ball_pos).norm().dot((robot.pose.position() - ball_pos).norm());
        curr_dp *= curr_dp;
        if (curr_dp > max) {
            max = curr_dp;
        }
    }

    // Additional heuristics for calculating optimal point
    double ball_proximity_loss = (current_point - ball_pos).mag() * .002;
    double goal_distance_loss = (9.0 - current_point.y()) * .008;

    //Final evaluation
    return max + ball_proximity_loss + goal_distance_loss + min_path_dist + min_robot_dist;
}

void Offense::receive_communication_response(communication::AgentPosResponseWrapper response) {
    Position::receive_communication_response(response);

    // Check to see if we are dealing with scorer requests
    if (const communication::ScorerRequest* scorer_response =
            std::get_if<communication::ScorerRequest>(&response.associated_request)) {
        handle_scorer_response(response.responses);
        return;
    }
}

communication::PosAgentResponseWrapper Offense::receive_communication_request(
    communication::AgentPosRequestWrapper request) {
    communication::PosAgentResponseWrapper comm_response =
        Position::receive_communication_request(request);

    // If a scorer request was received override the position receive_communication_request return
    if (const communication::ScorerRequest* scorer_request =
            std::get_if<communication::ScorerRequest>(&request.request)) {
        communication::ScorerResponse scorer_response = receive_scorer_request(*scorer_request);
        comm_response.response = scorer_response;
    } else if (const communication::ResetScorerRequest* _ =
                   std::get_if<communication::ResetScorerRequest>(&request.request)) {
        communication::Acknowledge response = receive_reset_scorer_request();
        comm_response.response = response;
    } else if (const communication::PassRequest* pass_request =
                   std::get_if<communication::PassRequest>(&request.request)) {
        //If the robot recieves a PassRequest, only process it if we are oppen
        
        rj_geometry::Point robot_position = last_world_state_->get_robot(true, robot_id_).pose.position();
        rj_geometry::Point from_robot_position = last_world_state_->get_robot(true, pass_request->from_robot_id).pose.position();
        rj_geometry::Segment pass_path{from_robot_position, robot_position};
        double min_robot_dist = 10000;
        float min_path_dist = 10000;

        //Calculates the minimum distance from the current robot to all other robots
        //Also calculates the minimum distance from another robot to the passing line
        for (auto bot : last_world_state_->their_robots) {
            rj_geometry::Point opp_pos = bot.pose.position();
            min_robot_dist = std::min(min_robot_dist, robot_position.dist_to(opp_pos));
            min_path_dist = std::min(min_path_dist, pass_path.dist_to(opp_pos));
        }

        //If the current robot is far enough away from other robots and there are no other robots in the passing line, process the request
        //Currently, max_receive_distance is used to determine when we are open, but this may need to change
        if (min_robot_dist > max_receive_distance && min_path_dist > max_receive_distance) {
            communication::PassResponse response = Position::receive_pass_request(*pass_request);
            comm_response.response = response;
        }
    }

    return comm_response;
}

void Offense::send_scorer_request() {
    communication::ScorerRequest scorer_request{};
    communication::generate_uid(scorer_request);
    scorer_request.robot_id = robot_id_;

    // Calculate distance to ball
    rj_geometry::Point robot_position =
        last_world_state_->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = last_world_state_->ball.position;
    double ball_distance = robot_position.dist_to(ball_position);
    scorer_request.ball_distance = ball_distance;

    communication::PosAgentRequestWrapper communication_request{};
    communication_request.request = scorer_request;
    communication_request.broadcast = true;

    communication_request_ = communication_request;
}

void Offense::send_reset_scorer_request() {
    communication::ResetScorerRequest reset_scorer_request{};
    communication::generate_uid(reset_scorer_request);

    communication::PosAgentRequestWrapper communication_request{};
    communication_request.request = reset_scorer_request;
    communication_request.broadcast = true;

    communication_request_ = communication_request;
    last_scorer_ = true;
    scorer_ = false;
}

communication::ScorerResponse Offense::receive_scorer_request(
    communication::ScorerRequest scorer_request) {
    communication::ScorerResponse scorer_response{};
    communication::generate_uid(scorer_response);
    scorer_response.robot_id = robot_id_;

    // Calculate distance to ball
    rj_geometry::Point robot_position =
        last_world_state_->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = last_world_state_->ball.position;
    double ball_distance = robot_position.dist_to(ball_position);
    scorer_response.ball_distance = ball_distance;

    // Switch scorers if better scorer
    if (scorer_ && scorer_request.ball_distance < ball_distance) {
        scorer_ = false;
        current_state_ = FACING;
    }

    // Give fake answer if previous scorer
    if (last_scorer_) {
        scorer_response.ball_distance = 300;
    }

    return scorer_response;
}

communication::Acknowledge Offense::receive_reset_scorer_request() {
    communication::Acknowledge acknowledge{};
    communication::generate_uid(acknowledge);

    last_scorer_ = false;
    send_scorer_request();

    return acknowledge;
}

void Offense::handle_scorer_response(
    const std::vector<communication::AgentResponseVariant>& responses) {
    rj_geometry::Point this_robot_position =
        last_world_state_->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = last_world_state_->ball.position;
    double this_ball_distance = this_robot_position.dist_to(ball_position);

    for (communication::AgentResponseVariant response : responses) {
        if (const communication::ScorerResponse* scorer_response =
                std::get_if<communication::ScorerResponse>(&response)) {
            if (scorer_response->ball_distance < this_ball_distance) {
                return;
            }
        }
    }

    // Make this robot the scorer
    scorer_ = true;
}

void Offense::derived_acknowledge_pass() { current_state_ = FACING; }

void Offense::derived_pass_ball() { current_state_ = PASSING; }

void Offense::derived_acknowledge_ball_in_transit() {
    current_state_ = RECEIVING;
    chasing_ball = false;
}

}  // namespace strategy
