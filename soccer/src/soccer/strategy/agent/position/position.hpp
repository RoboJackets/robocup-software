#pragma once

#include <spdlog/spdlog.h>

#include <rj_common/time.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/msg/empty_motion_command.hpp>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rj_msgs/action/robot_move.hpp"
#include "world_state.hpp"

namespace strategy {
/*
 * Position is an abstract superclass. Its subclasses handle strategy logic.
 * The goal is to isolate the strategy logic from the ROS interfacing. This
 * allows us to swap Position classes at runtime. (Google "Strategy Design
 * Pattern" for more info.)
 *
 * A good analogy is how the Planner Node uses the various Planner objects
 * (PathTargetPlanner, etc.). The Planner objects take in a plan request and
 * output a Trajectory. This class is sadly more coupled with the ActionClient
 * it lives in than the Planner objects, but this is necessary to have a
 * flexible agent that still has access to ROS info.
 */
class Position {
public:
    Position(int r_id);
    virtual ~Position() = default;

    Position(Position&&) noexcept = default;
    Position& operator=(Position&&) noexcept = default;
    Position(const Position&) = default;
    Position& operator=(const Position&) = default;

    // communication with AC
    void tell_is_done();
    void tell_time_left(double time_left);
    void tell_goal_canceled();
    void update_world_state(WorldState world_state);

    virtual rj_msgs::msg::RobotIntent get_task() = 0;

protected:
    // should be overriden in subclass constructors
    std::string position_name_{"Position"};

    // field for tell_time_left() above
    double time_left_{};

    // make WorldState thread-safe
    WorldState last_world_state_;
    mutable std::mutex world_state_mutex_;
    /*
     * @return thread-safe ptr to most recent world_state
     */
    [[nodiscard]] WorldState* world_state();

    /*
     * @brief assert world_state is valid before using it in get_task().
     *
     * Since ROS nodes launch in a random order, often the agents will launch
     * before they have any world_state info to act on. Thus, we must return
     * NOPs to the robots until vision_filter node starts up.
     *
     * @param intent EmptyMotionCommand added if world_state is invalid
     * @return false if world_state is invalid (nullptr), true otherwise
     */
    bool assert_world_state_valid(rj_msgs::msg::RobotIntent& intent);

    /*
     * @brief getter for is_done that clears the flag before returning
     * @return value of is_done before being cleared
     */
    bool check_is_done();
    bool is_done_{};

    /*
     * @brief getter for goal_canceled that clears the flag before returning
     * @return value of goal_canceled before being cleared
     */
    bool check_goal_canceled();
    bool goal_canceled_{};

    // const because should never be changed, but initializer list will allow
    // us to set this once initially
    const int robot_id_;

private:
};

}  // namespace strategy
