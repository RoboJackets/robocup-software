#pragma once

#include <chrono>
#include <cmath>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_common/planning/instant.hpp>


#pragma once

#include <cmath>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_common/field_dimensions.hpp>
#include <rj_common/planning/instant.hpp>
#include <rj_constants/constants.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>


#include <rj_common/time.hpp>
#include <rj_constants/constants.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/action/robot_move.hpp>
#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include "rj_strategy/agent/position.hpp"
#include "rj_strategy/agent/position/offense.hpp"

namespace strategy {

    class Runner : public Offense {
        public:
            ~Runner() override = default;
            Runner(int r_id);
            Runner(const Position& other);

        private:
            enum State {
                CENTER,
                DRIVING_TO_TOP_LEFT,
                DRIVING_TO_TOP_RIGHT,
                DRIVING_TO_BOTTOM_RIGHT,
                DRIVING_TO_BOTTOM_LEFT,
                TOP_LEFT,
                TOP_RIGHT,
                BOTTOM_RIGHT,
                BOTTOM_LEFT
            };

            State next_state();
            State current_state_ = State::CENTER;
            string state_to_name(State state){
                switch(state){
                    case CENTER:
                        return "Center";
                    case DRIVING_TO_TOP_LEFT:
                        return "Driving to top left";
                    case DRIVING_TO_TOP_RIGHT:
                        return "Driving to top right";
                    case DRIVING_TO_BOTTOM_RIGHT:
                        return "Driving to bottom right";
                    case DRIVING_TO_BOTTOM_LEFT:
                        return "Driving to bottom left";
                    case TOP_LEFT:
                        return "Top left";
                    case TOP_RIGHT:
                        return "Top right";
                    case BOTTOM_LEFT:
                        return "Bottom left";
                    case BOTTOM_RIGHT:
                        return "Bottom right";
                }
            }
            std::optional<RobotIntent> state_to_task(RobotIntent intent);
            std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;


    };
}