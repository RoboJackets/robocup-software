#pragma once

#include <cmath>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_common/field_dimensions.hpp>
#include <rj_common/planning/instant.hpp>
#include <rj_common/time.hpp>
#include <rj_constants/constants.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/action/robot_move.hpp>

#include "rj_strategy/agent/position.hpp"
#include "rj_strategy/agent/position/marker.hpp"
#include "rj_strategy/agent/position/waller.hpp"

namespace strategy {

/*
 * The Defense position handles general defensive tasks, like intercepting
 * passes, walling in front of our goal, and fighting for possession.
 */
class Defense : public Position {
public:
    Defense(int r_id);
    ~Defense() override = default;
    Defense(Position&& other);

    void receive_communication_response(communication::AgentPosResponseWrapper response) override;
    communication::PosAgentResponseWrapper receive_communication_request(
        communication::AgentPosRequestWrapper request) override;

    void derived_acknowledge_pass() override;
    void derived_pass_ball() override;
    void derived_acknowledge_ball_in_transit() override;
    std::string get_current_state() override;

    void die() override;
    void revive() override;

private:
    static constexpr int kMaxWallers{2};
    static constexpr RJ::Seconds kMarkingGroupJoinTimeout{2.0};
    static constexpr float kMarkingDistanceFactor{0.55f};

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
        IDLING,            // simply staying in place
        JOINING_WALL,      // send message to find its place in the wall
        WALLING,           // participating in the wall
        SEARCHING,         // moving around on the field to do something
        RECEIVING,         // physically intercepting the ball from a pass
        PASSING,           // physically kicking the ball towards another robot
        FACING,            // turning to face the passing robot
        MARKING,           // Following closely to an offense robot
        ENTERING_MARKING,  // Choosing/waiting for a robot to mark
    };

    State update_state();
    State current_state_ = IDLING;
    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    int get_marker_target_id();
    Marker marker_;
};

}  // namespace strategy
