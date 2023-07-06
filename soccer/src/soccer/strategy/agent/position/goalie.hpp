#pragma once

#include <cmath>

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
 * The Goalie position handles goalie behavior: blocking shots, passing to teammates, and clearing
 * the ball.
 */
class Goalie : public Position {
public:
    Goalie(int r_id);
    ~Goalie() override = default;

    void derived_acknowledge_pass() override;
    void derived_pass_ball() override;
    void derived_acknowledge_ball_in_transit() override;

private:
    // point goalie will aim for when clearing balls
    const rj_geometry::Point clear_point_{0.0, 4.5};

    // temp
    int send_idle_ct_ = 0;

    // see Position superclass
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    // possible states of the Goalie
    enum State {
        NOT_IN_BOX,
        IDLING,
        BLOCKING,        // blocking the ball from reaching the goal
        CLEARING,        // clearing the ball out of the goal box
        PREPARING_SHOT,  // pivot around ball in preparation for shot
        BALL_NOT_FOUND,  // the ball is not in play
    };

    bool check_ball_in_box(WorldState* world_state) {
        bool retval = false;

        rj_geometry::Point ball_pt = world_state->ball.position;
        bool geo_ball_in_box = this->field_dimensions_.our_defense_area().contains_point(ball_pt);

        if (geo_ball_in_box && !ball_in_box_start_time_valid_) {
            // when timer hasn't started but ball in box, start it
            ball_in_box_start_time_ = RJ::now();
            ball_in_box_start_time_valid_ = true;
        } else {
            // when timer started
            if (geo_ball_in_box) {
                // if in box, check for duration
                if (RJ::now() - ball_in_box_start_time_ > max_ball_in_duration) {
                    retval = true;
                }
            } else {
                // when ball leaves box, reset timer
                ball_in_box_start_time_valid_ = false;
            }
        }
        return retval;
    }
    RJ::Time ball_in_box_start_time_ = RJ::now();
    bool ball_in_box_start_time_valid_ = false;
    RJ::Seconds max_ball_in_duration = RJ::Seconds(0.2);

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
    State latest_state_ = BALL_NOT_FOUND;
};

}  // namespace strategy
