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
#include "role_interface.hpp"

namespace strategy {

/*
 * The Waller role provides the implementation for a defensive robot that implements
 * this class to have a waller-like behavior where it aims to serve as a wall between
 * the ball and the goal.
 */
class Zoner : public Position {
public:
    Zoner(int r_id);
    ~Zoner() override = default;
    Zoner(const Position& other);
    Zoner(Zoner&& other) = default;
    Zoner& operator=(const Zoner& other) = default;
    Zoner& operator=(Zoner&& other) = default;

private:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    // static constexpr double robot_diameter_multiplier_ = 1.5;
    enum State { ZONE, WALL };

    State current_state_ = ZONE;

    /**
     * @return what the state should be right now. called on each get_task tick
     */
    State next_state();

    /**
     * @return the task to execute. called on each get_task tick AFTER next_state()
     */
    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    static rj_geometry::Point find_centroid(const std::vector<rj_geometry::Point> opp_poses);

    std::string get_current_state() override;

    static constexpr double kZonerRadius = 1.5;
};

}  // namespace strategy
