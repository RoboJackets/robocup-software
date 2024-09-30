#pragma once

#include <cmath>
#include <string>

#include <spdlog/spdlog.h>

#include <rj_common/time.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>

#include "position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rj_msgs/action/robot_move.hpp"

namespace strategy {

/*
 * The Runner position handles Runner behavior: blocking shots, passing to teammates, and clearing
 * the ball.
 */
class Runner : public Position {
public:
    Runner(int r_id);
    ~Runner() override = default;
    Runner(const Position& other);

    void derived_acknowledge_pass() override;
    void derived_pass_ball() override;
    void derived_acknowledge_ball_in_transit() override;

    std::string get_current_state() override;

private:
    // point Runner will aim for when clearing balls
    const rj_geometry::Point clear_point_{0.0, 4.5};

    // temp
    int send_idle_ct_ = 0;

    // see Position superclass
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    // possible states of the Runner
    enum State {
        IDLING,          // doing nothing
        BLOCKING,        // blocking the ball from reaching the goal
        CLEARING,        // clearing the ball out of the goal box
        PREPARING_SHOT,  // pivot around ball in preparation for shot
        BALL_NOT_FOUND,  // the ball is not in play
        RECEIVING,       // physically intercepting the ball from a pass
        PASSING,         // physically kicking the ball at another robot
        FACING,          // turning to face the passing robot
        PENALTY,         // stay on baseline for penalty kick
        SIDE0,
        SIDE1,
        SIDE2,
        SIDE3
    };

    /*
     * @return true if ball is heading towards goal at some minimum speed threshold
     */
    bool shot_on_goal_detected(WorldState* world_state);

    /*
     * Provide an updated state for the Runner state machine based
     * on the current world state.
     *
     * e.g. when shot on goal detected, Runner should intercept the
     * ball
     *
     * @return current State to set Runner's state machine to
     */
    State update_state();

    /*
     * Based on the Runner's current state, send a motion_command
     * to the planner node.
     *
     * @param intent RobotIntent to add the desired motion_command
     * to
     *
     * @return matching return type of derived_get_task() (see
     * above)
     */
    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    // current state of Runner (state machine)
    State latest_state_ = IDLING;

    rj_geometry::Point penalty_location();
};

}  // namespace strategy
