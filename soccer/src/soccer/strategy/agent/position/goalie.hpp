#pragma once

#include <cmath>

#include <spdlog/spdlog.h>

#include <rj_common/time.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/msg/empty_motion_command.hpp>

#include "position.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rj_msgs/action/robot_move.hpp"

namespace strategy {

/*
 * The Defense position handles goalie behavior: blocking shots, passing to teammates, and clearing
 * the ball.
 */
class Goalie : public Position {
public:
    Goalie(int r_id);
    ~Goalie() override = default;

    rj_msgs::msg::RobotIntent get_task() override;

    void receive_communication_response(rj_msgs::msg::AgentToPosCommResponse response) override;
    rj_msgs::msg::PosToAgentCommResponse receive_communication_request(rj_msgs::msg::AgentToPosCommRequest request) override;

private:
    // temp to move back and forth
    int move_ct = 0;

    void set_position_request();
    void set_test_request();

    /*
     * @return Point for Goalie to block a shot. Calls get_idle_pt() if ball is
     * slow or shot will miss the goal.
     */
    rj_geometry::Point get_block_pt(WorldState* world_state) const;

    /*
     * @return Point for Goalie to stand in when no shot is coming. Expects
     * ball to be slow.
     */
    rj_geometry::Point get_idle_pt(WorldState* world_state) const;
};

}  // namespace strategy
