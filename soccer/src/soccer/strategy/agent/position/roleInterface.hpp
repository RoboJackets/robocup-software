#pragma once

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

namespace strategy {

/**
 * This abstract class serves as an interface for all roles (ie. Waller, Marker, Seeker, etc.) to
 * follow when being created.
 */
class RoleInterface {
public:
    /**
     * @brief Returns a behavior based on the role
     *
     * @param [RobotIntent intent] [RobotIntent of the Robot]
     * @return [RobotIntent with next target point for the robot]
     */
    virtual std::optional<RobotIntent> get_task(RobotIntent intent,
                                                rj_geometry::Point ball_location) = 0;

private:
};

}  // namespace strategy
