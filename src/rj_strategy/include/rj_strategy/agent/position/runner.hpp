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

namespace strategy {

/*
 * The Runner position handles runner behavior: moving in a square pattern around the field.
 */
class Runner : public Position {
public:
    Runner(int r_id);
    ~Runner() override = default;
    Runner(const Position& other);

    std::string get_current_state() override;

private:
    // possible states of the Runner
    enum State {
        MOVE_LEFT,  // moving to first corner
        MOVE_UP_FIELD,  // moving to second corner
        MOVE_RIGHT,  // moving to third corner
        MOVE_DOWN_FIELD,  // moving to fourth corner
    };

    rj_geometry::Point get_corner_point(State state);
    State update_state();
    std::optional<RobotIntent> state_to_task(RobotIntent intent);
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
    State latest_state_ = MOVE_LEFT;
    static constexpr double CORNER_ARRIVAL_THRESHOLD = 0.1;  // meters
};

}  // namespace strategy
