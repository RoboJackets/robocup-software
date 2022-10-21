#pragma once

#include <spdlog/spdlog.h>

#include <rj_common/time.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/msg/empty_motion_command.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rj_msgs/action/robot_move.hpp"

namespace strategy {
/*
 * Position handles strategy logic (currently handled by behavior tree). The
 * goal is to isolate the strategy from the ROS interfacing, so we can swap
 * Position classes fluidly.
 *
 * A good analogy is how the Planner Node uses the various Planner objects
 * (PathTargetPlanner, etc.). The Planner objects take in a plan request and
 * output a Trajectory. This class is more coupled with the ActionClient it
 * lives in, but this is necessary to have a flexible agent while still giving
 * access to ROS features.
 */
class Position {
public:
    Position();

    // communication with AC
    // TODO: heavily coupled, how fix?
    void tell_done();
    void tell_time_left(RJ::Seconds time_left);
    void tell_goal_canceled();

    rj_msgs::msg::RobotIntent get_task();

protected:
private:
    std::string position_name_;

    bool is_done_;
    RJ::Seconds time_left_;
    bool goal_canceled_;
};
}  // namespace strategy
