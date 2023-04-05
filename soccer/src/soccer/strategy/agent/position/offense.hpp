#pragma once

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/action/robot_move.hpp>

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

    void derived_acknowledge_pass() override;
    void derived_pass_ball() override;
    void derived_acknowledge_ball_in_transit() override;

private:
    bool kicking_{true};

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
    // TODO (Kevin): strategy design pattern for BallHandler/Receiver

    enum State {
        IDLING,     // simply staying in place
        SEARCHING,  // moving around on the field to get open
        PASSING,    // physically kicking the ball towards another robot
        SHOOTING,   // physically kicking the ball towards the net
        RECEIVING,  // physically intercepting the ball from a pass (gets possession)
        STEALING,   // attempting to intercept the ball from the other team
        FACING,
    };

    State update_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    // current state of the offensive agent (state machine)
    State current_state_ = IDLING;
};

}  // namespace strategy
