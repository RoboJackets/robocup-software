#pragma once

#include <cmath>

#include <spdlog/spdlog.h>

#include <rj_common/time.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/msg/empty_motion_command.hpp>
#include <rj_msgs/msg/goalie_idle_motion_command.hpp>
#include <rj_msgs/msg/path_target_motion_command.hpp>

#include "position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rj_msgs/action/robot_move.hpp"

namespace strategy {

/*
 * The Goalie position handles goalie behavior: blocking shots, passing to teammates, and clearing
 * the ball.
 */
class Goalie : public Position {
public:
    Goalie(int r_id);
    ~Goalie() override = default;

    rj_msgs::msg::RobotIntent get_task() override;

private:
    // temp
    int send_idle_ct_ = 0;

    /*
     * @return Point for Goalie to block a shot.
     *
     * Assumes ball is not slow.
     */
    static rj_geometry::Point get_block_pt(WorldState* world_state, bool& needs_to_block);
};

}  // namespace strategy
