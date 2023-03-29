#include "goalie.hpp"

namespace strategy {

Goalie::Goalie(int r_id) : Position(r_id) { position_name_ = "Goalie"; }

std::optional<RobotIntent> Goalie::derived_get_task(RobotIntent intent) {
    latest_state_ = update_state();
    return state_to_task(intent);
}

Goalie::State Goalie::update_state() {
    // if a shot is coming, override all and go block it
    WorldState* world_state = this->world_state();

    // if no ball found, stop and return to box immediately
    if (!world_state->ball.visible) {
        return BALL_NOT_FOUND;
    }

    // if a shot is coming, override all and go block it
    if (shot_on_goal_detected(world_state)) {
        return BLOCKING;
    }

    // if the ball is in the goalie box, clear it
    bool ball_is_slow = world_state->ball.velocity.mag() < 0.5;  // m/s

    rj_geometry::Point ball_pt = world_state->ball.position;

    bool ball_in_box = this->field_dimensions_.our_defense_area().contains_point(ball_pt);
    if (ball_is_slow && ball_in_box) {
        return CLEARING;
    }

    // otherwise, default to idling
    return IDLING;
}

std::optional<RobotIntent> Goalie::state_to_task(RobotIntent intent) {
    if (latest_state_ == BLOCKING) {
        planning::LinearMotionInstant target{rj_geometry::Point{0.0, 0.1}};
        auto intercept_cmd = planning::MotionCommand{"intercept", target};
        intent.motion_command = intercept_cmd;
        return intent;
    } else if (latest_state_ == IDLING) {
        auto goalie_idle_cmd = planning::MotionCommand{"goalie_idle"};
        intent.motion_command = goalie_idle_cmd;
        return intent;
    } else if (latest_state_ == CLEARING) {
        planning::LinearMotionInstant target{rj_geometry::Point{0.0, 4.5}};
        auto line_kick_cmd = planning::MotionCommand{"line_kick", target};
        intent.motion_command = line_kick_cmd;

        // note: the way this is set up makes it impossible to
        // shoot on time without breakbeam
        // TODO(Kevin): make intent hold a manip msg instead? to be cleaner?
        intent.shoot_mode = RobotIntent::ShootMode::CHIP;
        intent.trigger_mode = RobotIntent::TriggerMode::ON_BREAK_BEAM;
        intent.kick_speed = 4.0;
        intent.is_active = true;

        return intent;
    } else if (latest_state_ == BALL_NOT_FOUND) {
        // TODO: make point dependent on team
        rj_geometry::Point target_pt = this->field_dimensions_.our_defense_area().center();
        rj_geometry::Point target_vel{0.0, 0.0};

        planning::PathTargetFaceOption face_option = planning::FaceTarget{};

        // ball not found
        bool ignore_ball = true;

        planning::LinearMotionInstant target{target_pt, target_vel};
        intent.motion_command =
            planning::MotionCommand{"path_target", target, face_option, ignore_ball};
        return intent;
    }

    // should be impossible to reach, but this is equivalent to
    // sending an empty MotionCommand
    return std::nullopt;
}

bool Goalie::shot_on_goal_detected(WorldState* world_state) {
    rj_geometry::Point ball_pos = world_state->ball.position;
    rj_geometry::Point ball_vel = world_state->ball.velocity;

    // find x-coord that the ball would cross on the goal line to figure out if
    // shot is on target ((0, 0) is our goal, +y points out of goal)
    //
    // assumes ball vel will remain constant
    // TODO(Kevin): account for acceleration?
    if (ball_vel.y() == 0) {
        return false;
    }
    double time_to_cross = std::abs(ball_pos.y() / ball_vel.y());
    double cross_x = ball_pos.x() + ball_vel.x() * time_to_cross;

    bool shot_on_target = std::abs(cross_x) < this->field_dimensions_.goal_width() / 2.0;

    bool ball_is_fast = ball_vel.mag() > 1.0;
    return ball_is_fast && shot_on_target;
}

void Goalie::receive_communication_response(communication::AgentPosResponseWrapper response) {
    for (u_int32_t i = 0; i < response.responses.size(); i++) {
        if (const communication::Acknowledge* acknowledge =
                std::get_if<communication::Acknowledge>(&response.responses[i])) {
            SPDLOG_INFO("\033[92m Robot {} has acknowledged the message\033[0m",
                        response.received_robot_ids[i]);
        } else if (const communication::PassResponse* pass_response =
                       std::get_if<communication::PassResponse>(&response.responses[i])) {
            SPDLOG_INFO("\033[93m Robot {} has responded to the pass request\033[0m",
                        response.received_robot_ids[i]);
        } else if (const communication::PositionResponse* position_response =
                       std::get_if<communication::PositionResponse>(&response.responses[i])) {
            SPDLOG_INFO("\033[93m Robot {} is playing {}\033[0m", response.received_robot_ids[i],
                        position_response->position);
        } else if (const communication::TestResponse* test_response =
                       std::get_if<communication::TestResponse>(&response.responses[i])) {
            SPDLOG_INFO("Robot {} sent the test response '{}'", response.received_robot_ids[i],
                        test_response->message);
        } else {
            SPDLOG_WARN("\033[92mROBOT {} HAS SENT AN UNKNOWN RESPONSE\033[0m",
                        response.received_robot_ids[i]);
        }
    }
}

communication::PosAgentResponseWrapper Goalie::receive_communication_request(
    communication::AgentPosRequestWrapper request) {
    communication::PosAgentResponseWrapper comm_response{};
    if (const communication::TestRequest* test_request =
            std::get_if<communication::TestRequest>(&request.request)) {
        communication::TestResponse test_response{};
        test_response.message = fmt::format("The goalie (robot: {}) says hello", robot_id_);
        communication::generate_uid(test_response);
        comm_response.response = test_response;
    } else if (const communication::PositionRequest* position_request =
                   std::get_if<communication::PositionRequest>(&request.request)) {
        communication::PositionResponse position_response{};
        position_response.position = position_name_;
        communication::generate_uid(position_response);
        comm_response.response = position_response;
    } else if (const communication::PassRequest* pass_request =
                   std::get_if<communication::PassRequest>(&request.request)) {
        // TODO (https://app.clickup.com/t/8677c0q36): handle a pass request
        communication::Acknowledge acknowledge{};
        communication::generate_uid(acknowledge);
        comm_response.response = acknowledge;
    } else {
        communication::Acknowledge acknowledge{};
        communication::generate_uid(acknowledge);
        comm_response.response = acknowledge;
    }
    return comm_response;
}

}  // namespace strategy
