#pragma once

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/action/robot_move.hpp>
#include <rj_msgs/msg/empty_motion_command.hpp>

#include "planning/instant.hpp"
#include "position.hpp"
#include "rj_common/field_dimensions.hpp"
#include "rj_common/time.hpp"
#include "rj_constants/constants.hpp"
#include "rj_geometry/geometry_conversions.hpp"
#include "rj_geometry/point.hpp"

namespace strategy {

/*
 * The Waller role provides the implementation for a defensive robot that implements
 * this class to have a waller-like behavior where it aims to serve as a wall between
 * the ball and the goal.
 */
class Waller {
public:
    Waller(int waller_num);
    ~Waller() = default;

    /**
     * @brief Currently returns a waller behavior which aims to intercept the path
     * between the ball and the center of the goal
     *
     * @param [RobotIntent intent] [RobotIntent of the Defensive Robot]
     * @return [RobotIntent with next target point for the robot]
     */
    std::optional<RobotIntent> get_task(RobotIntent intent, rj_geometry::Point ball_location);

private:
    std::string defense_type;
    int waller_pos;
};

}  // namespace strategy
