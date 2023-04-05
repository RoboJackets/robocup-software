#pragma once

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_constants/constants.hpp>
#include <rj_msgs/action/robot_move.hpp>
#include <rj_msgs/msg/collect_motion_command.hpp>
#include <rj_msgs/msg/empty_motion_command.hpp>
#include <rj_msgs/msg/intercept_motion_command.hpp>
#include <rj_msgs/msg/settle_motion_command.hpp>

#include "planning/instant.hpp"
#include "position.hpp"
#include "rj_common/time.hpp"
#include "rj_geometry/geometry_conversions.hpp"
#include "rj_geometry/point.hpp"

namespace strategy {

/*
 * The Offense position handles general offensive tasks, like passing to
 * teammates, moving to get open, or shooting on goal.
 */
class Offense : public Position {
public:
    Offense(int r_id);
    ~Offense() override = default;

    void receive_communication_response(communication::AgentPosResponseWrapper response) override;
    communication::PosAgentResponseWrapper receive_communication_request(
        communication::AgentPosRequestWrapper request) override;

private:
    bool kicking_{true};

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
    // TODO (Kevin): strategy design pattern for BallHandler/Receiver
};

}  // namespace strategy
