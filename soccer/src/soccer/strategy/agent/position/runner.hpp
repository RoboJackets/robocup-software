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


#include "planning/instant.hpp"
#include "rj_common/field_dimensions.hpp"
#include "rj_common/time.hpp"
#include "rj_constants/constants.hpp"
#include "rj_geometry/geometry_conversions.hpp"
#include "rj_geometry/point.hpp"
#include "waller.hpp"

namespace strategy {
    class Runner : public Position {
        public:
        Runner(int r_id);
        ~Runner() override = default;
        
        
        private:

        enum State {
            TOP,
            LEFT,
            BOTTOM,
            RIGHT
        };

        std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

        State update_state();

        std::optional<RobotIntent> state_to_task(RobotIntent intent);

        State currentState_ = BOTTOM;

        rj_geometry::Point bottomLeft_ = rj_geometry::Point{3.0, 9.0};
        rj_geometry::Point bottomRight_ = rj_geometry::Point{-3.0, 9.0};
        rj_geometry::Point topLeft_ = rj_geometry::Point{3.0, 0.0};
        rj_geometry::Point topRight_ = rj_geometry::Point{-3.0, 0.0};
    };
}