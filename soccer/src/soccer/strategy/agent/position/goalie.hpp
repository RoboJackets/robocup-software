#pragma once

#include <cmath>

#include <spdlog/spdlog.h>

#include <rj_common/time.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/msg/empty_motion_command.hpp>
#include <rj_msgs/msg/goalie_idle_motion_command.hpp>
#include <rj_msgs/msg/intercept_motion_command.hpp>
#include <rj_msgs/msg/path_target_motion_command.hpp>

#include "planning/planner/intercept_planner.hpp"
#include "position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rj_msgs/action/robot_move.hpp"

namespace strategy {

/*
 * The Goalie position handles goalie behavior: blocking shots, passing to teammates, and clearing
 * the ball.
 */
class Goalie : public Position {
public:
    Goalie(int r_id);
    ~Goalie() override = default;

    communication::Acknowledge acknowledge_pass(communication::IncomingPassRequest incoming_pass_request) override;
    void pass_ball(int robot_id) override;
    communication::Acknowledge acknowledge_ball_in_transit(communication::BallInTransitRequest ball_in_transit_request) override;

private:
    // temp
    int send_idle_ct_ = 0;

    // see Position superclass
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    // possible states of the Goalie
    enum State {
        IDLING, // doing nothing
        BLOCKING, // blocking the ball from reaching the goal
        CLEARING, // clearing the ball out of the goal box
        BALL_NOT_FOUND, // the ball is not in play
        RECEIVING, // physically intercepting the ball from a pass
        PASSING, // physically kicking the ball at another robot
        FACING, // turning to face the passing robot
    };

    /*
     * @return true if ball is heading towards goal at some minimum speed threshold
     */
    bool shot_on_goal_detected(WorldState* world_state);

    /*
     * Provide an updated state for the Goalie state machine based
     * on the current world state.
     *
     * e.g. when shot on goal detected, Goalie should intercept the
     * ball
     *
     * @return current State to set Goalie's state machine to
     */
    State update_state();

    /*
     * Based on the Goalie's current state, send a motion_command
     * to the planner node.
     *
     * @param intent RobotIntent to add the desired motion_command
     * to
     *
     * @return matching return type of derived_get_task() (see
     * above)
     */
    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    // current state of Goalie (state machine)
    State latest_state_ = IDLING;
};

}  // namespace strategy
