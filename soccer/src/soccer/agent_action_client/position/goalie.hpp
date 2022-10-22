#pragma once

#include <spdlog/spdlog.h>

#include <rj_common/time.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/msg/empty_motion_command.hpp>

#include "position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rj_msgs/action/robot_move.hpp"

namespace strategy {

class Goalie : public Position {
public:
    Goalie();

    rj_msgs::msg::RobotIntent get_task() override;

protected:
private:
    // temp
    int move_ct = 0;
};

}  // namespace strategy
