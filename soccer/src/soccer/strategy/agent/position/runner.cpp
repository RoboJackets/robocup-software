#include "runner.hpp"

namespace strategy {

Runner::Runner(int r_id) : Position(r_id) {
    position_name_ = "Runner";
    current_state_ = IDLING; 
}

std::optional<RobotIntent> Runner::derived_get_task(RobotIntent intent) {
    current_state_ = update_state();
    return state_to_task(intent);
}

Runner::State Runner::update_state() {
    State next_state = current_state_;
    // handle transitions between current state
    WorldState* world_state = this->world_state();

    //SPDLOG_INFO("ANSON: test test test");

    // if no ball found, stop and return to box immediately
    if (!world_state->ball.visible) {
        return current_state_;
    }

    rj_geometry::Point robot_position = world_state->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = world_state->ball.position;
    double distance_to_ball = robot_position.dist_to(ball_position);

    if (current_state_ == IDLING) {
        //send_scorer_request();
        next_state = MOVING_LSIDE;
    } else if (current_state_ == MOVING_LSIDE && at_corner) {
        next_state = MOVING_USIDE;
        at_corner = false;
    } else if (current_state_ == MOVING_USIDE && at_corner) {
        next_state = MOVING_RSIDE;
        at_corner = false;
    } else if (current_state_ == MOVING_RSIDE && at_corner) {
        next_state = MOVING_DSIDE;
        at_corner = false;
    } else if (current_state_ == MOVING_DSIDE && at_corner) {
        next_state = MOVING_LSIDE;
        at_corner = false;
    } 
    return next_state;
}

std::optional<RobotIntent> Runner::state_to_task(RobotIntent intent) {
    SPDLOG_INFO("state: %d", current_state_);
    if (current_state_ == IDLING) {
        // Do nothing
        SPDLOG_INFO("runner: idling");
        auto empty_motion_cmd = planning::MotionCommand{};
        intent.motion_command = empty_motion_cmd;
        return intent;
    } else if (current_state_ == MOVING_LSIDE) {
        // DEFINE SEARCHING BEHAVIOR
        SPDLOG_INFO("runner: moving");
        rj_geometry::Point target_pt = rj_geometry::Point{1, 5};
        rj_geometry::Point target_vel{0.0, 0.0};
        planning::LinearMotionInstant target{target_pt, target_vel};
        rj_geometry::Point robot_position =
            world_state()->get_robot(true, robot_id_).pose.position();
        double dist_to_target = robot_position.dist_to(target_pt);
        SPDLOG_INFO("dist: %lf", dist_to_target);
        if (dist_to_target < 0.1) {
            SPDLOG_INFO("at target for LSIDE");
            at_corner = true;
        } 
        SPDLOG_INFO("at corner? %s", at_corner?"true":"false");
        //SPDLOG_INFO("robot position: " + robot_position);
        auto move_cmd = planning::MotionCommand{"path_target", target};
        intent.motion_command = move_cmd;
        return intent;
    } else if (current_state_ == MOVING_USIDE) {
        // DEFINE SEARCHING BEHAVIOR
        SPDLOG_INFO("runner: moving");
        rj_geometry::Point target_pt = rj_geometry::Point{2, 5};
        rj_geometry::Point target_vel{0.0, 0.0};
        planning::LinearMotionInstant target{target_pt, target_vel};
        rj_geometry::Point robot_position =
            world_state()->get_robot(true, robot_id_).pose.position();
        double dist_to_target = robot_position.dist_to(target_pt);
        SPDLOG_INFO("dist: %lf", dist_to_target);
        if (dist_to_target < 0.1) {
            SPDLOG_INFO("at target for USIDE");
            at_corner = true;
        } 
        SPDLOG_INFO("at corner? %s", at_corner?"true":"false");
        //SPDLOG_INFO("robot position: " + robot_position);
        auto move_cmd = planning::MotionCommand{"path_target", target};
        intent.motion_command = move_cmd;
        return intent;
    } else if (current_state_ == MOVING_RSIDE) {
        // DEFINE SEARCHING BEHAVIOR
        SPDLOG_INFO("runner: moving");
        rj_geometry::Point target_pt = rj_geometry::Point{2, 4};
        rj_geometry::Point target_vel{0.0, 0.0};
        planning::LinearMotionInstant target{target_pt, target_vel};
        rj_geometry::Point robot_position =
            world_state()->get_robot(true, robot_id_).pose.position();
        double dist_to_target = robot_position.dist_to(target_pt);
        SPDLOG_INFO("dist: %lf", dist_to_target);
        if (dist_to_target < 0.1) {
            SPDLOG_INFO("at target for RSIDE");
            at_corner = true;
        } 
        SPDLOG_INFO("at corner? %s", at_corner?"true":"false");
        //SPDLOG_INFO("robot position: " + robot_position);
        auto move_cmd = planning::MotionCommand{"path_target", target};
        intent.motion_command = move_cmd;
        return intent;
    } else if (current_state_ == MOVING_DSIDE) {
        // DEFINE SEARCHING BEHAVIOR
        SPDLOG_INFO("runner: moving");
        rj_geometry::Point target_pt = rj_geometry::Point{1, 4};
        rj_geometry::Point target_vel{0.0, 0.0};
        planning::LinearMotionInstant target{target_pt, target_vel};
        rj_geometry::Point robot_position =
            world_state()->get_robot(true, robot_id_).pose.position();
        double dist_to_target = robot_position.dist_to(target_pt);
        SPDLOG_INFO("dist: %lf", dist_to_target);
        if (dist_to_target < 0.1) {
            SPDLOG_INFO("at target for DSIDE");
            at_corner = true;
        } 
        SPDLOG_INFO("at corner? %s", at_corner?"true":"false");
        //SPDLOG_INFO("robot position: " + robot_position);
        auto move_cmd = planning::MotionCommand{"path_target", target};
        intent.motion_command = move_cmd;
        return intent;
    } 

    // should be impossible to reach, but this is an EmptyMotionCommand
    return std::nullopt;
}

void Runner::receive_communication_response(communication::AgentPosResponseWrapper response) {
    Position::receive_communication_response(response);

    // Check to see if we are dealing with scorer requests
    if (const communication::ScorerRequest* scorer_response =
            std::get_if<communication::ScorerRequest>(&response.associated_request)) {
        handle_scorer_response(response.responses);
        return;
    }
}

communication::PosAgentResponseWrapper Runner::receive_communication_request(
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
    }

    return comm_response;
}

void Runner::send_scorer_request() {
    communication::ScorerRequest scorer_request{};
    communication::generate_uid(scorer_request);
    scorer_request.robot_id = robot_id_;

    // Calculate distance to ball
    rj_geometry::Point robot_position = world_state()->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = world_state()->ball.position;
    double ball_distance = robot_position.dist_to(ball_position);
    scorer_request.ball_distance = ball_distance;

    communication::PosAgentRequestWrapper communication_request{};
    communication_request.request = scorer_request;
    communication_request.broadcast = true;

    communication_request_ = communication_request;
}

void Runner::send_reset_scorer_request() {
    communication::ResetScorerRequest reset_scorer_request{};
    communication::generate_uid(reset_scorer_request);

    communication::PosAgentRequestWrapper communication_request{};
    communication_request.request = reset_scorer_request;
    communication_request.broadcast = true;

    communication_request_ = communication_request;
    last_scorer_ = true;
    scorer_ = false;
}

communication::ScorerResponse Runner::receive_scorer_request(
    communication::ScorerRequest scorer_request) {
    communication::ScorerResponse scorer_response{};
    communication::generate_uid(scorer_response);
    scorer_response.robot_id = robot_id_;

    // Calculate distance to ball
    rj_geometry::Point robot_position = world_state()->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = world_state()->ball.position;
    double ball_distance = robot_position.dist_to(ball_position);
    scorer_response.ball_distance = ball_distance;

    // Switch scorers if better scorer
    if (scorer_ && scorer_request.ball_distance < ball_distance) {
        scorer_ = false;
        current_state_ = IDLING;
    }

    // Give fake answer if previous scorer
    if (last_scorer_) {
        scorer_response.ball_distance = 300;
    }

    return scorer_response;
}

communication::Acknowledge Runner::receive_reset_scorer_request() {
    communication::Acknowledge acknowledge{};
    communication::generate_uid(acknowledge);

    last_scorer_ = false;
    send_scorer_request();

    return acknowledge;
}

void Runner::handle_scorer_response(
    const std::vector<communication::AgentResponseVariant>& responses) {
    rj_geometry::Point this_robot_position =
        world_state()->get_robot(true, robot_id_).pose.position();
    rj_geometry::Point ball_position = world_state()->ball.position;
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

void Runner::derived_acknowledge_pass() { current_state_ = IDLING; }

void Runner::derived_pass_ball() { current_state_ = IDLING; }

void Runner::derived_acknowledge_ball_in_transit() {
    current_state_ = IDLING;
    chasing_ball = false;
}

}