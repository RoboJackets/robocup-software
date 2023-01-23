#include "offense.hpp"

namespace strategy {

Offense::Offense(int r_id) : Position(r_id) { position_name_ = "Offense"; }

std::optional<RobotIntent> Offense::derived_get_task(RobotIntent intent) {
    // FSM: kick -> move away -> repeat
    if (check_is_done()) {
        // switch from kicking -> not kicking or vice versa
        kicking_ = !kicking_;
    }

    // get world_state
    WorldState* world_state = this->world_state();  // thread-safe getter

    rj_geometry::Point ball_pos = world_state->ball.position;
    rj_geometry::Point ball_vel = world_state->ball.velocity;
    rj_geometry::Point robot_pos = world_state->our_robots[robot_id_].pose.position();
    auto msg = rj_msgs::msg::RobotStatus::SharedPtr;

    float SETTLE_BALL_SPEED_THRESHOLD = 0.75;

    if ("Receive") {
        //Fix Imports

        //Do seek behavior

        bool ball_slow = ball_vel.norm() < .05;
        double dist_to_ball = (robot_pos - ball_pos).norm();
        bool ball_close = dist_to_ball - (kRobotRadius + kBallRadius) < 0.03;

        if (!(msg->has_ball_sense || ball_vel.norm() < SETTLE_BALL_SPEED_THRESHOLD)) {

            //Settle Ball
            auto settle_cmd = planning::SettleMotionCommand{};
            intent.motion_command = settle_cmd;
            intent.motion_command_name = "settle";
            intent.is_active = true;
            intent.dribbler_speed = 1.0;
            return intent;

        } else if (!(ball_slow && ball_close) {

            //Capture Ball
            auto collect_cmd = planning::CollectMotionCommand{};
            intent.motion_command = collect_cmd;
            intent.motion_command_name = "collect";
            intent.dribbler_speed = 1.0;
            intent.is_active = true;
            return intent;

        } else {

            return std:nullopt;

        }
    }
}

void Offense::receive_communication_response(communication::AgentPosResponseWrapper response) {
    for (u_int32_t i = 0; i < response.responses.size(); i++) {
        if (const communication::Acknowledge* acknowledge =
                std::get_if<communication::Acknowledge>(&response.responses[i])) {
            SPDLOG_INFO("\033[93mRobot {} has acknowledged the message: {}\033[0m",
                        response.received_robot_ids[i]);
        } else if (const communication::PassResponse* pass_response =
                       std::get_if<communication::PassResponse>(&response.responses[i])) {
            SPDLOG_INFO("\033[92mRobot {} has sent the pass response\033[0m",
                        response.received_robot_ids[i]);
        } else if (const communication::TestResponse* test_response =
                       std::get_if<communication::TestResponse>(&response.responses[i])) {
            SPDLOG_INFO("\033[91mRobot {} has sent a test response with message: {}\033[0m",
                        response.received_robot_ids[i], test_response->message);
        } else if (const communication::PositionResponse* position_response =
                       std::get_if<communication::PositionResponse>(&response.responses[i])) {
            SPDLOG_INFO("\033[90mRobot {} has sent a position response\033[0m",
                        response.received_robot_ids[i]);
        }
    }
}

communication::PosAgentResponseWrapper Offense::receive_communication_request(
    communication::AgentPosRequestWrapper request) {
    communication::PosAgentResponseWrapper comm_response{};
    if (const communication::PassRequest* pass_request =
            std::get_if<communication::PassRequest>(&request.request)) {
        // TODO (https://app.clickup.com/t/8677c0q36): Handle pass requests
        sleep(100);
        communication::Acknowledge acknowledge{};
        communication::generate_uid(acknowledge);
        comm_response.response = acknowledge;
    } else if (const communication::PositionRequest* position_request =
                   std::get_if<communication::PositionRequest>(&request.request)) {
        communication::PositionResponse position_response{};
        position_response.position = position_name_;
        communication::generate_uid(position_response);
        comm_response.response = position_response;
    } else if (const communication::TestRequest* test_request =
                   std::get_if<communication::TestRequest>(&request.request)) {
        communication::TestResponse test_response{};
        test_response.message = fmt::format("An offensive player (robot: {}) says hi", robot_id_);
        communication::generate_uid(test_response);
        comm_response.response = test_response;
    } else {
        communication::Acknowledge acknowledge{};
        communication::generate_uid(acknowledge);
        comm_response.response = acknowledge;
    }
    return comm_response;
}

}  // namespace strategy
