#pragma once

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_common/field_dimensions.hpp>
#include <rj_common/planning/instant.hpp>
#include <rj_common/time.hpp>
#include <rj_constants/constants.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/action/robot_move.hpp>

#include "rj_strategy/agent/position.hpp"
#include "rj_strategy/agent/position/role_interface.hpp"

namespace strategy {

class Runner : public Position {
public:
    Runner(int r_id);
    ~Runner() override = default;
    Runner(const Position& other);
    Runner(Runner&& other) = default;
    Runner& operator=(const Runner& other) = default;
    Runner& operator=(Runner&& other) = default;

private:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    // enum State { SIDE1, SIDE2, SIDE3, SIDE4 };
    vector<int> states_ = {0, 1, 2, 3};

    rj_geometry::Point point1{1.0, 5.0};
    rj_geometry::Point point2{2.0, 5.0};
    rj_geometry::Point point3{2.0, 6.0};
    rj_geometry::Point point4{1.0, 6.0};

    vector<rj_geometry::Point> corners_ = {point1, point2, point3, point4};
    int current_state_ = states_[0];

    int next_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);
    std::string get_current_state() override;
};
}  // namespace strategy