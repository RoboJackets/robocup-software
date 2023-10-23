#pragma once
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/action/robot_move.hpp>

#include "planning/instant.hpp"
#include "position.hpp"
#include "rj_common/time.hpp"
#include "rj_geometry/geometry_conversions.hpp"
#include "rj_geometry/point.hpp"

namespace strategy {

class Runner : public Position {
public:
    Runner(int r_id);
    ~Runner() override = default;

private:
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    enum State { RUNNING_LEFT, RUNNING_TOP, RUNNING_RIGHT, RUNNING_BOTTOM };

    State update_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    State current_state_;
};
}  // namespace strategy