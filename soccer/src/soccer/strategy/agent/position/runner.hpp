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
    enum State { LEFT_SIDE, TOP_SIDE, RIGHT_SIDE, BOTTOM_SIDE };

    State current_state_ = LEFT_SIDE;

    const rj_geometry::Point top_left_target_{2.83, 1.39};
    const rj_geometry::Point top_right_target_{-2.83, 1.39};
    const rj_geometry::Point bottom_right_target_{-2.83, 7.7};
    const rj_geometry::Point bottom_left_target_{2.83, 7.7};

    State update_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
};

}  // namespace strategy