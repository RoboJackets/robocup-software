#pragma once

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/action/robot_move.hpp>

#include "planning/instant.hpp"
#include "position.hpp"
#include "rj_common/field_dimensions.hpp"
#include "rj_common/time.hpp"
#include "rj_constants/constants.hpp"
#include "rj_geometry/geometry_conversions.hpp"
#include "rj_geometry/point.hpp"
#include "waller.hpp"

namespace strategy {

/*
 * The Defense position handles general defensive tasks, like intercepting
 * passes, walling in front of our goal, and fighting for possession.
 */
class Defense : public Position {
public:
    Defense(int r_id);
    ~Defense() override = default;

    communication::Acknowledge acknowledge_pass(
        communication::IncomingPassRequest incoming_pass_request) override;
    void pass_ball(int robot_id) override;
    communication::Acknowledge acknowledge_ball_in_transit(
        communication::BallInTransitRequest ball_in_transit_request) override;

    communication::Acknowledge acknowledge_pass(communication::IncomingPassRequest incoming_pass_request) override;
    void pass_ball(int robot_id) override;

private:
    int move_ct_ = 0;

    /**
     * @brief The derived_get_task method returns the task for the defensive robot
     *  to do based on the game situation. The method will continuously look to assign
     *  the robot to a defensive role, allowing us to quickly swap roles through
     *  gametime. Roles aim to include things such as waller, blocker, etc.
     *
     * @param [RobotIntent intent] [RobotIntent of the Defensive Robot]
     * @return [RobotIntent with next target point for the robot]
     */
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
