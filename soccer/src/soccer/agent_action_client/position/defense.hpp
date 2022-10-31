#pragma once

#include <cmath>

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

// TODO: docstring
class Defense : public Position {
public:
    Defense();
    ~Defense() override = default;

    Defense(Defense&&) noexcept = default;
    Defense& operator=(Defense&&) noexcept = default;
    Defense(const Defense&) = default;
    Defense& operator=(const Defense&) = default;

    rj_msgs::msg::RobotIntent get_task() override;

protected:
private:
};

}  // namespace strategy
