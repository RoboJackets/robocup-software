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
 * The Offense position handles general offensive tasks, like passing to
 * teammates, moving to get open, or shooting on goal.
 */
class Offense : public Position {
public:
    Offense(int r_id);
    ~Offense() override = default;

    rj_msgs::msg::RobotIntent get_task() override;

    void receive_communication_response(rj_msgs::msg::AgentToPosCommResponse response) override;
    rj_msgs::msg::PosToAgentCommResponse receive_communication_request(rj_msgs::msg::AgentToPosCommRequest request) override;

private:
    bool kicking_{true};
    // TODO: strategy design pattern for BallHandler/Receiver
};

}  // namespace strategy
