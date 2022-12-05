#pragma once

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/action/robot_move.hpp>
#include <rj_msgs/msg/empty_motion_command.hpp>

#include "planning/instant.hpp"
#include "position.hpp"
#include "rj_common/time.hpp"
#include "rj_geometry/geometry_conversions.hpp"
#include "rj_geometry/point.hpp"

namespace strategy {

/*
 * The Defense position handles general defensive tasks, like intercepting
 * passes, walling in front of our goal, and fighting for possession.
 */
class Defense : public Position {
public:
    Defense(int r_id);
    ~Defense() override = default;

    void receive_communication_response(rj_msgs::msg::AgentToPosCommResponse response) override;
    rj_msgs::msg::PosToAgentCommResponse receive_communication_request(
        rj_msgs::msg::AgentToPosCommRequest request) override;

private:
    void set_test_multicast_request();
    int move_ct_ = 0;

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
};

}  // namespace strategy
