#include "defense.hpp"

namespace strategy {

Defense::Defense(int r_id) : Position(r_id) { position_name_ = "Defense"; }

std::optional<RobotIntent> Defense::derived_get_task(RobotIntent intent) {

    // thread-safe getter
    WorldState* world_state = this->world_state();

    // Get Ball Location
    rj_geometry::Point ball_location{world_state->ball.position};

    // Create Waller
    Waller waller{0};
    return waller.get_task(intent, ball_location);
}

void Defense::receive_communication_response(communication::AgentPosResponseWrapper response) {
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
            SPDLOG_INFO("\033[92m Robot {} sent the test response {}\033[0m",
                        response.received_robot_ids[i], test_response->message);
        } else {
            SPDLOG_WARN("\033[92mROBOT {} HAS SENT AN UNKNOWN RESPONSE\033[0m",
                        response.received_robot_ids[i]);
        }
    }
}

communication::PosAgentResponseWrapper Defense::receive_communication_request(
    communication::AgentPosRequestWrapper request) {
    communication::PosAgentResponseWrapper comm_response;
    if (const communication::PassRequest* pass_request =
            std::get_if<communication::PassRequest>(&request.request)) {
        // TODO (https://app.clickup.com/t/8677c0q36): Handle pass response
        sleep(5);
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
        test_response.message = fmt::format("robot {} says hello", robot_id_);
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
