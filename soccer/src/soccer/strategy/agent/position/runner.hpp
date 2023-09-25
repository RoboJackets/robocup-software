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
public
    Runner(int r_id);
    ~Runner() override = default;

private:
    std::optional<RobotIntent> derived_get_task(RobotIntnt intent) override;

    enum State {
        RUNFIRSTSIDE,
        RUNSECONDSIDE,
        RUNTHIRDSIDE,
        RUNFOURTHSIDE,
        IDLING,
    };

    State update_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    rj_geometry::Point initial_position;

    State latest_state_ = IDLING;

    double side_length = 50;
};

}  // namespace strategy
