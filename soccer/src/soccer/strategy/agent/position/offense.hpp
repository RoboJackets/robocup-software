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
 * The Offense position handles general offensive tasks, like passing to
 * teammates, moving to get open, or shooting on goal.
 */
class Offense : public Position {
public:
    Offense(int r_id);
    ~Offense() override = default;

    void receive_communication_response(communication::AgentPosResponseWrapper response) override;
    communication::PosAgentResponseWrapper receive_communication_request(
        communication::AgentPosRequestWrapper request) override;

    communication::Acknowledge acknowledge_pass(communication::IncomingPassRequest incoming_pass_request) override;
    void pass_ball(int robot_id) override;
    communication::Acknowledge acknowledge_ball_in_transit(communication::BallInTransitRequest ball_in_transit_request) override;

private:
    bool kicking_{true};

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
    // TODO (Kevin): strategy design pattern for BallHandler/Receiver

    enum State {
        IDLING, // simply staying in place
        SEARCHING, // moving around on the field to get open
        PASSING, // physically kicking the ball towards another robot
        SHOOTING, // physically kicking the ball towards the net
        RECEIVING, // physically intercepting the ball from a pass (gets possession)
        STEALING, // attempting to intercept the ball from the other team
        FACING,
    };

    State update_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    // current state of the offensive agent (state machine)
    State current_state_ = IDLING;

    double BALL_RECEIVE_DISTANCE = 0.1;
    double BALL_LOST_DISTANCE = 0.5;
};

}  // namespace strategy
