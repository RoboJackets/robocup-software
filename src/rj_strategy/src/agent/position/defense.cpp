#include "rj_strategy/agent/position/defense.hpp"

namespace strategy {

Defense::Defense(int r_id) : Position(r_id, "Defense") {}

Defense::Defense(Position&& other) : Position{std::move(other)} {
    position_name_ = "Defense";
}

std::optional<RobotIntent> Defense::derived_get_task(RobotIntent intent) {
    current_state_ = update_state();

    return state_to_task(intent);
}

std::string Defense::get_current_state() {
    return std::string{"Defense"} + std::to_string(static_cast<int>(current_state_));
}

Defense::State Defense::update_state() {
    WorldState* world_state = last_world_state_;

    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = world_state->ball.position;
    double distance_to_ball = robot_position.dist_to(ball_position);

    // Update state based on coordinator async calls resolving
    State next_state = current_state_;

    // If we are the closest robot to a loose ball, break off whatever we are
    // doing and go steal it. Reachable from any state except the ball-handling
    // states themselves (STEALING/SHOOTING), so we don't interrupt a steal or
    // shot already in progress.
    if (current_state_ != STEALING && current_state_ != SHOOTING && can_steal_ball()) {
        // Give up any wall/marking slot we were holding before chasing the ball.
        client_handles_->waller->leave_group();
        client_handles_->marking->leave_group();
        SPDLOG_INFO("robot {} ", robot_id_);
        return STEALING;
    }

    switch (current_state_) {
        case IDLING:
            next_state = JOINING_WALL;
            break;
        case JOINING_WALL:
            client_handles_->waller->join_group([this](WallerClient::Result result) {
                if (result.success)
                    current_state_ = WALLING;
                else
                    current_state_ = ENTERING_MARKING;
            });
            break;
        case WALLING:
            if (!client_handles_->waller->am_i_member()) next_state = IDLING;
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
            break;
        case MARKING:
            if (!client_handles_->marking->am_i_member() ||
                !client_handles_->marking->am_i_marking()) {
                next_state = IDLING;
            }
            break;
        case ENTERING_MARKING: {
            if (!sent_join_marking_group_request_) {
                sent_join_marking_group_request_ = true;
                request_time_ = RJ::now();

                client_handles_->marking->join_group([this](const bool is_member) {
                    if (is_member) {
                        current_state_ = MARKING;
                    } else {
                        current_state_ = IDLING;
                    }
                });
            }
            auto elapsed = RJ::now() - request_time_;
            if (elapsed > kMarkingGroupJoinTimeout) {
                // reset flag
                sent_join_marking_group_request_ = false;
                // ensure not in coordinator group
                client_handles_->marking->leave_group();
                SPDLOG_INFO("Robot {}: Timeout on join group, IDLING now", robot_id_);
                next_state = IDLING;
            }

            break;
        }
        case STEALING:
            // Once we have possession of the ball, shoot it. SHOOTING is only
            // reachable from here.
            if (check_is_done() || distance_to_ball < kOwnBallRadius) {
                next_state = SHOOTING;
                break;
            }
            // If another robot became closer or the ball is no longer legally
            // accessible, give up the steal and return to normal defense.
            if (!can_steal_ball()) {
                next_state = IDLING;
            }
            break;
        case SHOOTING:
            // Once we have kicked or lost the ball, return to normal defense.
            if (check_is_done() || distance_to_ball > ball_lost_distance_ ||
                !ball_in_play_area(world_state, field_dimensions_)) {
                next_state = IDLING;
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
            // drive up to the ball (no dribbler available, so just approach it)
            chasing_ball = true;
            auto approach_ball_cmd = planning::MotionCommand{
                "path_target", planning::LinearMotionInstant{ball_position}, planning::FaceBall{}};
            intent.motion_command = approach_ball_cmd;
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
        auto walling_point =
            client_handles_->waller->get_walling_point(last_world_state_, field_dimensions_);
        if (walling_point) {
            planning::LinearMotionInstant target{walling_point.value()};
            intent.motion_command =
                planning::MotionCommand{"path_target", target, planning::FaceBall{}};
        } else
            intent.motion_command = planning::MotionCommand{};
        return intent;
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
        rj_geometry::Point targetPoint =
            last_world_state_->get_robot(false, client_handles_->marking->who_am_i_marking())
                .pose.position();

        rj_geometry::Point ballPoint = last_world_state_->ball.position;
        rj_geometry::Point targetToBall =
            (ballPoint - targetPoint).normalized(kMarkingDistanceFactor);
        planning::LinearMotionInstant goal{targetPoint + targetToBall};
        // SPDLOG_INFO("Location to mark: {}, {}", (targetPoint + targetToBall).x(), (targetPoint +
        // targetToBall).y());
        intent.motion_command =
            planning::MotionCommand{"path_target", goal, planning::FaceBall{}, true};

        return intent;
    } else if (current_state_ == STEALING) {
        // Drive to the ball to win possession, mirroring the Offense steal.
        auto collect_cmd = planning::MotionCommand{
            "path_target", planning::LinearMotionInstant{last_world_state_->ball.position},
            planning::FaceBall{}};
        intent.motion_command = collect_cmd;
        return intent;
    } else if (current_state_ == SHOOTING) {
        // Line kick towards the best shot on goal.
        planning::LinearMotionInstant target{
            calculate_best_shot(last_world_state_, field_dimensions_)};
        auto shoot_cmd =
            planning::MotionCommand{"line_kick", target, planning::FaceTarget{}, true};
        intent.motion_command = shoot_cmd;
        intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
        intent.kick_speed = max_kick_speed();
        return intent;
    }

    return std::nullopt;
}

void Defense::receive_communication_response(communication::AgentPosResponseWrapper response) {
    // Call to super
    Position::receive_communication_response(response);
}

communication::PosAgentResponseWrapper Defense::receive_communication_request(
    communication::AgentPosRequestWrapper request) {
    // Call to super
    communication::PosAgentResponseWrapper response =
        Position::receive_communication_request(request);

    // Return the response
    return response;
}

void Defense::derived_acknowledge_pass() { current_state_ = FACING; }

void Defense::derived_pass_ball() { current_state_ = PASSING; }

void Defense::derived_acknowledge_ball_in_transit() {
    current_state_ = RECEIVING;
    chasing_ball = false;
}

bool Defense::can_steal_ball() const {
    // If ball is not legally accessible, obviously can't steal
    if (!ball_in_play_area(last_world_state_, field_dimensions_)) {
        return false;
    }

    // Only steal when the other team actually has the ball; otherwise leave
    // ball-winning to our offense and stay in our defensive assignment.
    if (they_have_ball(last_world_state_)) {
        return false;
    }

    // Ball location
    rj_geometry::Point ball_position = this->last_world_state_->ball.position;

    // Our robot is closest robot to ball

    auto current_pos = last_world_state_->get_robot(true, robot_id_).pose.position();

    auto our_dist = (current_pos - ball_position).mag();

    for (auto pal : this->last_world_state_->our_robots) {
        auto dist = (pal.pose.position() - ball_position).mag();
        if (pal.visible && dist < our_dist) {
            return false;
        }
    }
    
    return true;
}

void Defense::die() { client_handles_->waller->leave_group(); }

void Defense::revive() { current_state_ = JOINING_WALL; }

}  // namespace strategy
