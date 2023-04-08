#include "offense.hpp"

namespace strategy {

Offense::Offense(int r_id) : Position(r_id) { position_name_ = "Offense"; }

std::optional<RobotIntent> Offense::derived_get_task(RobotIntent intent) {
    if (this->play_state_.is_our_restart() && this->play_state_.is_penalty()) {
        PenaltyPlayer player{};
        return player.get_task(intent, this->world_state(), this->field_dimensions_);
    } else if (this->play_state_.is_penalty_playing() && this->play_state_.is_our_restart()) {
        PenaltyKicker kicker{};
        return kicker.get_task(intent, this->world_state(), this->field_dimensions_);
    }

    // FSM: kick -> move away -> repeat
    if (check_is_done()) {
        // switch from kicking -> not kicking or vice versa
        kicking_ = !kicking_;
    }

    // get world_state
    WorldState* world_state = this->world_state();  // thread-safe getter

    // oscillate along vertical line (temp)
    double y = 6.0;
    if (!kicking_) {
        y = 1.0;
    }
    rj_geometry::Point target_pt{(6.0 * (robot_id_ * 0.1) - 2.0), y};

    // stop at end of path
    rj_geometry::Point target_vel{0.0, 0.0};

    // face ball on way up, face path on way down
    planning::PathTargetFaceOption face_option = planning::FaceBall{};
    if (kicking_) {
        face_option = planning::FaceTarget{};
    }

    // avoid ball
    bool ignore_ball = false;

    planning::LinearMotionInstant target{target_pt, target_vel};
    intent.motion_command =
        planning::MotionCommand{"path_target", target, face_option, ignore_ball};
    return intent;
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
