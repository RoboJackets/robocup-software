#pragma once

#include <cmath>

#include <spdlog/spdlog.h>

#include <rj_common/time.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>

#include "position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rj_msgs/action/robot_move.hpp"

namespace strategy {

class Runner : public Position {
public:
    Runner(int r_id);
    ~Runner() override = default;

private:

    enum State {
        LEFT,
        TOP,
        RIGHT,
        BOTTOM
    };

    State current_state_ = LEFT;

    State update_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
};

} // namespace strategy