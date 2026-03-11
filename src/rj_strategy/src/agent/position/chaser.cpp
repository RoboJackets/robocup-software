#include "rj_strategy/agent/position/chaser.hpp"

#include <limits>

#include <rj_geometry/segment.hpp>

namespace strategy {

Chaser::Chaser(int r_id) : Position{r_id, "Chaser"} {}

Chaser::Chaser(Position&& other) : Position{std::move(other)} { position_name_ = "Chaser"; }

std::optional<RobotIntent> Chaser::derived_get_task(RobotIntent intent) {
    const State new_state = next_state();
    if (new_state != current_state_) {
        reset_timeout();
        if (new_state != POSSESSION) {
            pass_request_sent_ = false;
        }
    }

    current_state_ = new_state;
    return state_to_task(intent);
}

std::string Chaser::get_current_state() {
    return std::string{"Chaser "} + std::string{state_to_name(current_state_)};
}

Chaser::State Chaser::next_state() {
    switch (current_state_) {
        case POSITIONING: {
            if (distance_to_ball() < kOwnBallRadius) {
                return POSSESSION;
            }

            if (should_press()) {
                return PRESSING;
            }

            return POSITIONING;
        }

        case PRESSING: {
            if (check_is_done() || distance_to_ball() < kOwnBallRadius) {
                return POSSESSION;
            }

            if (!should_press() || timed_out()) {
                return POSITIONING;
            }

            return PRESSING;
        }

        case POSSESSION: {
            if (distance_to_ball() > kBallLostDistance) {
                return POSITIONING;
            }

            return POSSESSION;
        }

        case PASSING: {
            if (check_is_done()) {
                pass_ball(pass_to_robot_id_);
                return POSITIONING;
            }

            if (distance_to_ball() > kBallLostDistance) {
                return POSITIONING;
            }

            return PASSING;
        }

        case RECEIVING: {
            if (check_is_done() && distance_to_ball() < kOwnBallRadius) {
                return POSSESSION;
            }

            if (timed_out()) {
                return POSITIONING;
            }

            return RECEIVING;
        }
    }

    return current_state_;
}

std::optional<RobotIntent> Chaser::state_to_task(RobotIntent intent) {
    switch (current_state_) {
        case POSITIONING: {
            rj_geometry::Point target = field_dimensions_.our_half().center();

            if (!ball_in_our_half()) {
                target = field_dimensions_.their_half().center();

                if (alive_robots_[offense_robot_id()]) {
                    const auto offense_pos =
                        last_world_state_->get_robot(true, offense_robot_id()).pose.position();
                    target = (target + offense_pos) / 2.0;
                }
            }

            planning::LinearMotionInstant goal{target, rj_geometry::Point{0.0, 0.0}};
            intent.motion_command =
                planning::MotionCommand{"path_target", goal, planning::FaceBall{}, true};
            return intent;
        }

        case PRESSING: {
            intent.motion_command = planning::MotionCommand{"collect"};
            return intent;
        }

        case POSSESSION: {
            if (timed_out()) {
                pass_request_sent_ = false;
                reset_timeout();
            }

            if (!pass_request_sent_ && alive_robots_[offense_robot_id()]) {
                send_direct_pass_request({static_cast<u_int8_t>(offense_robot_id())});
                pass_request_sent_ = true;
            }

            intent.motion_command = planning::MotionCommand{"collect"};
            return intent;
        }

        case PASSING: {
            const auto target_robot_pos =
                last_world_state_->get_robot(true, pass_to_robot_id_).pose.position();
            planning::LinearMotionInstant target{target_robot_pos};
            intent.motion_command = planning::MotionCommand{"line_kick", target};
            intent.shoot_mode = RobotIntent::ShootMode::KICK;
            intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
            intent.kick_speed = 4.0;
            intent.is_active = true;
            return intent;
        }

        case RECEIVING: {
            if (distance_to_ball() > max_receive_distance && !chasing_ball) {
                const auto robot_position =
                    last_world_state_->get_robot(true, robot_id_).pose.position();
                planning::LinearMotionInstant stay_put{
                    robot_position, rj_geometry::Point{0.0, 0.0}};
                intent.motion_command =
                    planning::MotionCommand{"path_target", stay_put, planning::FaceBall{}};
            } else {
                chasing_ball = true;
                intent.motion_command = planning::MotionCommand{"collect"};
            }
            return intent;
        }
    }

    return std::nullopt;
}

communication::PosAgentResponseWrapper Chaser::receive_communication_request(
    communication::AgentPosRequestWrapper request) {
    communication::PosAgentResponseWrapper comm_response =
        Position::receive_communication_request(request);

    if (const communication::PassRequest* pass_request =
            std::get_if<communication::PassRequest>(&request.request)) {
        auto response = Position::receive_pass_request(*pass_request);
        if (current_state_ == POSITIONING && check_if_open(pass_request->from_robot_id)) {
            response.direct_open = true;
        }
        comm_response.response = response;
    }

    return comm_response;
}

void Chaser::receive_communication_response(communication::AgentPosResponseWrapper response) {
    for (u_int32_t i = 0; i < response.responses.size(); i++) {
        if (const communication::Acknowledge* acknowledge =
                std::get_if<communication::Acknowledge>(&response.responses[i])) {
            if (const communication::IncomingBallRequest* incoming_ball_request =
                    std::get_if<communication::IncomingBallRequest>(&response.associated_request)) {
                (void)acknowledge;
                (void)incoming_ball_request;
                current_state_ = PASSING;
                pass_to_robot_id_ = response.received_robot_ids[i];
                reset_timeout();
            }
        } else if (const communication::PassResponse* pass_response =
                       std::get_if<communication::PassResponse>(&response.responses[i])) {
            if (const communication::PassRequest* sent_pass_request =
                    std::get_if<communication::PassRequest>(&response.associated_request)) {
                if (sent_pass_request->direct && pass_response->direct_open) {
                    send_pass_confirmation(response.received_robot_ids[i]);
                }
            }
        }
    }
}

void Chaser::derived_acknowledge_pass() {
    current_state_ = RECEIVING;
    chasing_ball = false;
    pass_request_sent_ = false;
    reset_timeout();
}

void Chaser::derived_pass_ball() { pass_request_sent_ = false; }

void Chaser::derived_acknowledge_ball_in_transit() {
    current_state_ = RECEIVING;
    chasing_ball = false;
    pass_request_sent_ = false;
    reset_timeout();
}

bool Chaser::ball_in_our_half() const {
    return field_dimensions_.our_half().contains_point(last_world_state_->ball.position);
}

bool Chaser::should_press() const {
    if (!last_world_state_->ball.visible) {
        return false;
    }

    const auto ball_position = last_world_state_->ball.position;
    if (!ball_in_our_half() || field_dimensions_.our_defense_area().contains_point(ball_position)) {
        return false;
    }

    const double our_dist = distance_to_ball();
    double best_other_dist = std::numeric_limits<double>::infinity();

    for (int i = 0; i < static_cast<int>(kNumShells); ++i) {
        if (i == robot_id_ || i == goalie_id_ || !alive_robots_[i]) {
            continue;
        }

        const double dist =
            last_world_state_->get_robot(true, i).pose.position().dist_to(ball_position);
        best_other_dist = std::min(best_other_dist, dist);
    }

    return our_dist <= kPressBallRadius || our_dist <= best_other_dist;
}

bool Chaser::check_if_open(int from_robot_id) const {
    const auto robot_position = last_world_state_->get_robot(true, robot_id_).pose.position();
    const auto from_robot_position =
        last_world_state_->get_robot(true, from_robot_id).pose.position();
    const rj_geometry::Segment pass_path{from_robot_position, robot_position};

    double min_robot_dist = std::numeric_limits<double>::infinity();
    double min_path_dist = std::numeric_limits<double>::infinity();

    for (const auto& bot : last_world_state_->their_robots) {
        const auto opp_pos = bot.pose.position();
        min_robot_dist = std::min(min_robot_dist, robot_position.dist_to(opp_pos));
        min_path_dist = std::min(min_path_dist, pass_path.dist_to(opp_pos));
    }

    return min_robot_dist > max_receive_distance && min_path_dist > max_receive_distance;
}

}  // namespace strategy
