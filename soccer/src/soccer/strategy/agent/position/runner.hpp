#pragma once

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/action/robot_move.hpp>

#include "planning/instant.hpp"
#include "position.hpp"
#include "rj_common/field_dimensions.hpp"
#include "rj_common/time.hpp"
#include "rj_constants/constants.hpp"
#include "rj_geometry/geometry_conversions.hpp"
#include "rj_geometry/point.hpp"
#include "waller.hpp"

namespace strategy {

/*
 * The Defense position handles general defensive tasks, like intercepting
 * passes, walling in front of our goal, and fighting for possession.
 */
class Runner : public Position {
public:
    Runner(int r_id);
    ~Runner() override = default;

    void receive_communication_response(communication::AgentPosResponseWrapper response) override;
    communication::PosAgentResponseWrapper receive_communication_request(
        communication::AgentPosRequestWrapper request) override;

    void derived_acknowledge_pass() override;
    void derived_pass_ball() override;
    void derived_acknowledge_ball_in_transit() override;

    void die() override;
    void revive() override;

private:

    /**
     * @brief The derived_get_task method returns the task for the defensive robot
     *  to do based on the game situation. The method will continuously look to assign
     *  the robot to a defensive role, allowing us to quickly swap roles through
     *  gametime. Roles aim to include things such as waller, blocker, etc.
     *
     * @param [RobotIntent intent] [RobotIntent of the Defensive Robot]
     * @return [RobotIntent with next target point for the robot]
     */
    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

    enum State {
        LEFT,       
        DOWN,  
        RIGHT,       
        UP,     
    };

    State update_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    State current_state_ = LEFT;
};

}  // namespace strategy