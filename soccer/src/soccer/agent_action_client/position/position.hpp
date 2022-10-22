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
 * Position is an abstract superclass. Its subclasses handle strategy logic.
 * The goal is to isolate the strategy from the ROS interfacing, so we can swap
 * Position classes fluidly. (Google "Strategy Design Pattern".)
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
    void tell_is_done();
    void tell_time_left(double time_left);
    void tell_goal_canceled();

    virtual rj_msgs::msg::RobotIntent get_task() = 0;

protected:
    std::string position_name_;

    bool is_done_;
    double time_left_;
    bool goal_canceled_;

    // return value of is_done_ and set to false
    bool check_is_done();
    bool check_goal_canceled();

private:
};

}  // namespace strategy
