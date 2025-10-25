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

class Runner : public Position {
public:
    Runner(int r_id);
    ~Runner() override = default;
    Runner(const Position& other);

    std::string get_current_state() override;

private:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
    // possible states of the Runner
    enum State {
        IDLING,
        WALL1,          
        WALL2,       
        WALL3,  
        WALL4
    };
    std::optional<RobotIntent> state_to_task(RobotIntent intent);
    State next_state();

    State update_state();

    rj_geometry::Point Point1{-1, 4};
    rj_geometry::Point Point2{-1, 6};
    rj_geometry::Point Point3{2, 6};
    rj_geometry::Point Point4{2, 4};

    static constexpr float threshold_ = 0.1;

    State latest_state_ = WALL1;
};

}  // namespace strategy
