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
                IDLING,          // doing nothing
                UP,              // Traversing the top side of the shape
                LEFT,            // Left side, etc.
                DOWN,
                RIGHT
            };
            State latest_state_ = IDLING;
            rj_geometry::Point point_up = rj_geometry::Point(3.0, 0.0);
            rj_geometry::Point point_left = rj_geometry::Point(3.0, 9.0);
            rj_geometry::Point point_down = rj_geometry::Point(-3.0, 9.0);
            rj_geometry::Point point_right = rj_geometry::Point(-3.0, 0.0);
            std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

            State update_state();

            std::optional<RobotIntent> state_to_task(RobotIntent intent);

    };
}