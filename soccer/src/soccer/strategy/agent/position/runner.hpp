#pragma once

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
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    // did not know this was also in C lol
    enum State {
        RUN_FIRST_SIDE,
        RUN_SECOND_SIDE,
        RUN_THIRD_SIDE,
        RUN_FOURTH_SIDE,
        IDLING,
    };

    State update_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    rj_geometry::Point initial_position_;

    State latest_state_ = IDLING;

    constexpr static double kSideLength = 1.0;
};

}  // namespace strategy