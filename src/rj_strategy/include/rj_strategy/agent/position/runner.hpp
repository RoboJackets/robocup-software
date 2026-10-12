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
#include "rj_strategy/agent/position/seeker.hpp"
#include "rj_strategy/agent/position_utils.hpp"

namespace strategy {
    /**
     * This position involves a runner that will repeatedly move in a rectangle pattern.
    */
    class Runner : public Position {    
    public:
        Runner(int r_id);
        Runner(Position&&);
        ~Runner() = default;

    private:

        /**
        * @brief Calls next_state and state_to_task on each tick
        */
        std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

        enum State {
            DEFAULT,
            FIRST_SIDE,
            SECOND_SIDE,
            THIRD_SIDE,
            FOURTH_SIDE,
        };

        /**
        * @return what the state of the runner should be now
        */
        State next_state();

        /**
        * @return the task to currently execute given the robots state
        */
        std::optional<RobotIntent> state_to_task(RobotIntent intent);

        State current_state_ = State::DEFAULT;

        /**
        * @return if the position of the runner is at the edge of the first vertex
        */
        bool at_first_vertex();

        /**
        * @return if the position of the runner is at the edge of the second vertex
        */
        bool at_second_vertex();

        /**
        * @return if the position of the runner is at the edge of the third vertex
        */
        bool at_third_vertex();

        /**
        * @return if the position of the runner is at the edge of the fourth vertex
        */
        bool at_fourth_vertex();


    };
} // end of namespace strategy