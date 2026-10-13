#pragma once

#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_common/time.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/action/robot_move.hpp>

#include "rj_strategy/agent/position.hpp"
#include "rj_strategy/agent/position_utils.hpp"

namespace strategy {

/*
 * The Runner runs a rectangle
 */
class Runner : public Position {
public:
    Runner(int r_id);
    ~Runner() override = default;
    Runner(Position&& other);

    std::string get_current_state() override;

    std::string get_state_name() const override {
        return std::string(state_to_name(latest_state_));
    }

private:
    // 4 corners of a rectangle
    const rj_geometry::Point top_left_{1.0, 3.0};
    const rj_geometry::Point top_right_{-1.0, 3.0};
    const rj_geometry::Point bottom_left_{1.0, 6.0};
    const rj_geometry::Point bottom_right_{-1.0, 6.0};

    // see Position superclass
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    // possible states of the Runner
    enum State { TOP_LEFT, TOP_RIGHT, BOTTOM_LEFT, BOTTOM_RIGHT };

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
    State latest_state_ = TOP_LEFT;

    static constexpr std::string_view state_to_name(State s) {
        switch (s) {
            case TOP_LEFT:
                return "TOP_LEFT";
            case TOP_RIGHT:
                return "TOP_RIGHT";
            case BOTTOM_LEFT:
                return "BOTTOM_LEFT";
            case BOTTOM_RIGHT:
                return "BOTTOM_RIGHT";
        }
    }
};

}  // namespace strategy
