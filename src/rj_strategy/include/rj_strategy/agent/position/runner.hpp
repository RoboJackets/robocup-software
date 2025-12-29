#pragma once

#include <chrono>
#include <cmath>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_common/planning/instant.hpp>
#include <rj_common/time.hpp>
#include <rj_constants/constants.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/action/robot_move.hpp>

#include "rj_strategy/agent/position.hpp"

namespace strategy {
    class Runner : public Position {
        public:
            Runner(int r_id);
            ~Runner() override = default;
            Runner(const Position& other);

            communication::PosAgentResponseWrapper receive_communication_request(
                communication::AgentPosRequestWrapper request) override;

            void receive_communication_response(communication::AgentPosResponseWrapper response) override;

            std::string get_current_state() override;

        private:
            /**
             * @brief Overriden from Position. Calls next_state and then state_to_task on each tick.
             */
            std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

            // define corners of rectangular path
            const Point top_right{-2.5, 4.5};
            const Point top_left{2.5, 4.5};
            const Point bottom_left{2.5,-4.5};
            const Point bottom_right{-2.5,-4.5};

            enum State {
                UP,     // moving up towards top right
                LEFT,   // moving left towards top left
                DOWN, //   moving down towards bottom left
                RIGHT //   moving right towards bottom right
            };

            /**
             * @return what the state should be right now. called on each get_task tick
             */
            State next_state();

            /**
             * @return the task to execute. called on each get_task tick AFTER next_state()
             */
            std::optional<RobotIntent> state_to_task(RobotIntent intent);
            
            State current_state_ = State::UP;
        };
}