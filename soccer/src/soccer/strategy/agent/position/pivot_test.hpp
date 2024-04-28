#pragma once

#include <chrono>
#include <cmath>
#include <string>
#include <unordered_map>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/action/robot_move.hpp>

#include "planning/instant.hpp"
#include "position.hpp"
#include "rj_common/time.hpp"
#include "rj_constants/constants.hpp"
#include "rj_geometry/geometry_conversions.hpp"
#include "rj_geometry/point.hpp"

namespace strategy {

class Pivot : public Position {
public:
    Pivot(int r_id);
    ~Pivot() override = default;
    Pivot(const Pivot& other) = default;
    Pivot(Pivot&& other) = default;

    Pivot& operator=(const Pivot& other) = delete;
    Pivot& operator=(Pivot&& other) = delete;

    std::string get_current_state() override;

private:
    /**
     * @brief Overriden from Position. Calls next_state and then state_to_task on each tick.
     */
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    enum State { OUR_GOAL, OPP_GOAL, IDLE };

    State current_state_ = IDLE;

    rj_geometry::Point curr_pt_{};

    int count_ = 0;

    /**
     * @return what the state should be right now. called on each get_task tick
     */
    State next_state();

    /**
     * @return the task to execute. called on each get_task tick AFTER next_state()
     */
    std::optional<RobotIntent> state_to_task(RobotIntent intent);
};

}  // namespace strategy
