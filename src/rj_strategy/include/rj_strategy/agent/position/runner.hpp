#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_geometry/point.hpp>
#include <rj_msgs/action/robot_move.hpp>

#include "rj_strategy/agent/position.hpp"
#include "rj_strategy/agent/position_utils.hpp"

namespace strategy {

/**
 * The Runner position takes laps around the field in a rectangle.
 */
class Runner : public Position {
public:
    Runner(int r_id);
    ~Runner() override = default;
    Runner(Position&& other);

    std::string get_current_state() override;

    std::string get_state_name() const override {
        return std::string(state_to_name(current_state_));
    }

private:
    /**
     * @brief Overriden from Position. Calls next_state and then state_to_task on each tick.
     */
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    enum State {
        RUNNING_SIDE_1,  // corner_1_ -> corner_2_
        RUNNING_SIDE_2,  // corner_2_ -> corner_3_
        RUNNING_SIDE_3,  // corner_3_ -> corner_4_
        RUNNING_SIDE_4,  // corner_4_ -> corner_1_
    };

    State next_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    // Debugging
    static constexpr std::string_view state_to_name(State s) {
        switch (s) {
            case RUNNING_SIDE_1:
                return "RUNNING_SIDE_1";
            case RUNNING_SIDE_2:
                return "RUNNING_SIDE_2";
            case RUNNING_SIDE_3:
                return "RUNNING_SIDE_3";
            case RUNNING_SIDE_4:
                return "RUNNING_SIDE_4";
        }
        return "UNKNOWN";
    }

    State current_state_ = RUNNING_SIDE_1;

    // Corners of the rectangle path
    rj_geometry::Point corner_1_{-2.0, 2.0};
    rj_geometry::Point corner_2_{2.0, 2.0};
    rj_geometry::Point corner_3_{2.0, 6.0};
    rj_geometry::Point corner_4_{-2.0, 6.0};
};

}  // namespace strategy