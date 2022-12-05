#include "goalie.hpp"

namespace strategy {

// TODO: lock Goalie id to id given by the ref
Goalie::Goalie(int r_id) : Position(r_id) { 
    position_name_ = "Goalie";
    set_position_request();
}

rj_msgs::msg::RobotIntent Goalie::get_task() {
    // init an intent with our robot id
    rj_msgs::msg::RobotIntent intent;
    intent.robot_id = robot_id_;

    // if world_state invalid, return empty_intent
    if (!assert_world_state_valid()) {
        return get_empty_intent();
    }

    if (check_is_done()) {
        move_ct++;
    }

    // thread-safe getter
    WorldState* world_state = this->world_state();

    if (world_state == nullptr) {
        auto empty = rj_msgs::msg::EmptyMotionCommand{};
        intent.motion_command.empty_command = {empty};
    } else {
        auto ptmc = rj_msgs::msg::PathTargetMotionCommand{};
        auto pt = get_block_pt(world_state);
        ptmc.target.position = rj_convert::convert_to_ros(pt);

        rj_geometry::Point ball_pos = world_state->ball.position;
        auto face_pt = ball_pos;
        ptmc.override_face_point = {rj_convert::convert_to_ros(face_pt)};
        ptmc.ignore_ball = true;
        intent.motion_command.path_target_command = {ptmc};
    }
    // TODO: capture ball if vel is slow & ball is inside box
    // (waiting on field pts to be given to world_state)

    return intent;
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

void Goalie::receive_communication_response(rj_msgs::msg::AgentToPosCommResponse response) {
    if (response.response.response_type == 1) {
        SPDLOG_INFO("\033[92mRobot {} has sent the test response message: {}\033[0m", response.robot_id, response.response.test_response[0].message);
    }else if (response.response.response_type == 2) {
        switch (response.response.position_response[0].position) {
        case 1:
            SPDLOG_INFO("\033[93mRobot {} is playing defense\033[0m", response.robot_id);
            break;
        case 2:
            SPDLOG_INFO("\033[93mRobot {} is playing offense\033[0m", response.robot_id);
            break;
        default:
            SPDLOG_INFO("\033[93mRobot {} is playing an undefined role\033[0m", response.robot_id);
            break;
        }

        if (response.robot_id == 1) {
            set_test_request();
        }
    } else {
        SPDLOG_INFO("\033[92mRobot {} has acknowledged the message\033[0m", response.robot_id);
    }
}

rj_msgs::msg::PosToAgentCommResponse Goalie::receive_communication_request(rj_msgs::msg::AgentToPosCommRequest request) {
    rj_msgs::msg::PosToAgentCommResponse comm_response{};
    if (request.request.request_type == 1) {
        rj_msgs::msg::TestResponse test_response{};
        test_response.message = "The goalie has obtained your message";
        comm_response.response.test_response = {test_response};
        comm_response.response.response_type = 1;
    } else {
        rj_msgs::msg::Acknowledge acknowledge{};
        comm_response.response.acknowledge_response = {acknowledge};
        comm_response.response.response_type = 0;
    }
    return comm_response;
}

void Goalie::set_position_request() {
    rj_msgs::msg::PosToAgentCommRequest request;
    rj_msgs::msg::PositionRequest position_request{};
    request.broadcast = true;
    request.request.position_request = {position_request};
    request.request.request_type = 2;
    communication_request_ = request;
}

void Goalie::set_test_request() {
    rj_msgs::msg::PosToAgentCommRequest request;
    rj_msgs::msg::TestRequest test_request;
    request.num_targets = 1;
    request.target_agents = {2};
    request.request.test_request = {test_request};
    request.request.request_type = 1;
    communication_request_ = request;
}

}  // namespace strategy
