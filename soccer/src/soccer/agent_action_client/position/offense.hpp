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
class Offense : public Position {
public:
    Offense(int r_id);
    ~Offense() override = default;

    Offense(Offense&&) noexcept = default;
    Offense& operator=(Offense&&) noexcept = default;
    Offense(const Offense&) = default;
    Offense& operator=(const Offense&) = default;

    rj_msgs::msg::RobotIntent get_task() override;

protected:
private:
    bool kicking_{true};
    // TODO: strategy design pattern for BallHandler/Receiver
};

}  // namespace strategy
