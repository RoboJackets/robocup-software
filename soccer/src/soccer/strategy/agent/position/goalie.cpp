#include "goalie.hpp"

namespace strategy {

// TODO(Kevin): lock Goalie id to id given by the ref
Goalie::Goalie(int r_id) : Position(r_id) { position_name_ = "Goalie"; }

std::optional<rj_msgs::msg::RobotIntent> Goalie::get_task() {
    // init an intent with our robot id
    rj_msgs::msg::RobotIntent intent;
    intent.robot_id = robot_id_;

    // if world_state invalid, return empty_intent
    if (!assert_world_state_valid()) {
        return get_empty_intent();
    }

        WorldState* world_state = this->world_state();  // thread-safe getter
    if (shot_on_goal_detected(world_state)) {
        // TODO(Kevin): fix intercept planner's is_done, then add in logic to
        // clear/pass ball once done intercepting
        auto intercept_mc = rj_msgs::msg::InterceptMotionCommand{};
        // TODO(Kevin): once field added in use goal pos instead of hardcoding
        intercept_mc.target.x = 0.0;
        intercept_mc.target.y = 0.1;
        intent.motion_command.intercept_command = {intercept_mc};
        intent.motion_command.name = "intercept";
        return intent;
    } else {
        auto goalie_idle = rj_msgs::msg::GoalieIdleMotionCommand{};
        intent.motion_command.goalie_idle_command = {goalie_idle};
        intent.motion_command.name = "goalie_idle";
        return intent;
    }

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

    bool shot_on_target =
        std::abs(cross_x) < 0.5;  // TODO(Kevin): add field to world_state to avoid hardcoding this
    bool ball_is_fast = ball_vel.mag() > 1.0;
    return ball_is_fast && shot_on_target;
}

rj_geometry::Point Goalie::get_block_pt(WorldState* world_state) const {
    // TODO: make intercept planner do what its header file does, so we don't need this
    // also, fix the intercept planner so we don't have to pass in the ball
    // point every tick
    rj_geometry::Point ball_pos = world_state->ball.position;
    rj_geometry::Point ball_vel = world_state->ball.velocity;

    // if ball is slow, return the idle pt (no kick coming)
    if (ball_vel.mag() < 0.1) {
        return get_idle_pt(world_state);
    }

    // find x-coord that the ball would cross on the goal line
    // (0, 0) is our goal, +y points out of goal
    // assume ball vel will remain constant
    double time_to_cross = std::abs(ball_pos.y() / ball_vel.y());  // again, ball_vel is > 0
    double cross_x = ball_pos.x() + ball_vel.x() * time_to_cross;

    // if shot is going out of goal, ignore it
    // TODO: add field to world_state
    if (std::abs(cross_x) > 0.6) {
        return get_idle_pt(world_state);
    }

    rj_geometry::Point block_pt{cross_x, 0.0};
    return block_pt;
}

rj_geometry::Point Goalie::get_idle_pt(WorldState* world_state) const {
    // TODO: transfer field part of world_state
    // TODO: make this depend on team +/-x
    rj_geometry::Point ball_pos = world_state->ball.position;
    rj_geometry::Point goal_pt{0.0, 0.0};

    // TODO: move closer/farther from ball as a linear % of distance from ball
    double goalie_dist = 0.5;
    rj_geometry::Point idle_pt = (ball_pos - goal_pt).norm();
    idle_pt *= goalie_dist;
    // TODO: clamp y to 0

    return idle_pt;
}

void Goalie::receive_communication_response(communication::AgentPosResponseWrapper response) {
    if (response.response.response_type == communication::CommunicationType::test) {
        SPDLOG_INFO("\033[92mRobot {} has sent the test response message: {}\033[0m",
                    response.from_robot_id, response.response.test_response[0].message);
    } else if (response.response.response_type == communication::CommunicationType::position) {
        switch (response.response.position_response[0].position) {
            case 1:
                SPDLOG_INFO("\033[93mRobot {} is playing defense\033[0m", response.from_robot_id);
                break;
            case 2:
                SPDLOG_INFO("\033[93mRobot {} is playing offense\033[0m", response.from_robot_id);
                break;
            default:
                SPDLOG_INFO("\033[93mRobot {} is playing an undefined role\033[0m",
                            response.from_robot_id);
                break;
        }

        if (response.from_robot_id == 1) {
            set_test_request();
        }
    } else {
        SPDLOG_INFO("\033[92mRobot {} has acknowledged the message\033[0m", response.from_robot_id);
    }
}

communication::PosAgentResponseWrapper Goalie::receive_communication_request(
    communication::AgentPosRequestWrapper request) {
    communication::PosAgentResponseWrapper comm_response;
    if (request.request.request_type == communication::CommunicationType::test) {
        rj_msgs::msg::TestResponse test_response{};
        test_response.message = "The goalie has obtained your message";
        comm_response.response.test_response = {test_response};
        comm_response.response.response_type = communication::CommunicationType::test;
    } else {
        rj_msgs::msg::Acknowledge acknowledge{};
        comm_response.response.acknowledge_response = {acknowledge};
        comm_response.response.response_type = communication::CommunicationType::acknowledge;
    }
    return comm_response;
}

void Goalie::set_position_request() {
    communication::PosAgentRequestWrapper request;
    rj_msgs::msg::PositionRequest position_request{};
    request.broadcast = true;
    request.request.position_request = {position_request};
    request.request.request_type = communication::CommunicationType::position;
    communication_request_ = request;
}

void Goalie::set_test_request() {
    communication::PosAgentRequestWrapper request;
    rj_msgs::msg::TestRequest test_request;
    request.broadcast = false;
    request.target_agents = {2};
    request.request.test_request = {test_request};
    request.request.request_type = communication::CommunicationType::test;
    communication_request_ = request;
}

}  // namespace strategy
