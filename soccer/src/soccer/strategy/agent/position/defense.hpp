#pragma once

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/action/robot_move.hpp>
#include <rj_msgs/msg/empty_motion_command.hpp>

#include "planning/instant.hpp"
#include "position.hpp"
#include "rj_common/time.hpp"
#include "rj_geometry/geometry_conversions.hpp"
#include "rj_geometry/point.hpp"

namespace strategy {

/*
 * The Defense position handles general defensive tasks, like intercepting
 * passes, walling in front of our goal, and fighting for possession.
 */
class Defense : public Position {
public:
    Defense(int r_id);
    ~Defense() override = default;

    void receive_communication_response(communication::AgentPosResponseWrapper response) override;
    communication::PosAgentResponseWrapper receive_communication_request(
        communication::AgentPosRequestWrapper request) override;

    communication::Acknowledge acknowledge_pass(communication::IncomingPassRequest incoming_pass_request) override;
    void pass_ball(int robot_id) override;

private:
    int move_ct_ = 0;

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    enum State {
        IDLING, // simply staying in place
        SEARCHING, // moving around on the field to do something
        RECEIVING, // physically intercepting the ball from a pass
        PASSING, // physically kicking the ball towards another robot
    };

    State update_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    // current state of the defense agent (state machine)
    State current_state_ = IDLING;

    double BALL_RECEIVE_DISTANCE = 0.1;
    double BALL_LOST_DISTANCE = 0.5;
};

}  // namespace strategy
