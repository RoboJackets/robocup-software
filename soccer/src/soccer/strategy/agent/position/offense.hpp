#pragma once

#include <cmath>

#include <spdlog/spdlog.h>

#include <planning/instant.hpp>
#include <rj_common/time.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/msg/empty_motion_command.hpp>

#include "position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rj_msgs/action/robot_move.hpp"

namespace strategy {

/*
 * The Offense position handles general offensive tasks, like passing to
 * teammates, moving to get open, or shooting on goal.
 */
class Offense : public Position {
public:
    Offense(int r_id);
    ~Offense() override = default;

private:
    bool kicking_{true};
    // TODO: strategy design pattern for BallHandler/Receiver

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
};

}  // namespace strategy
