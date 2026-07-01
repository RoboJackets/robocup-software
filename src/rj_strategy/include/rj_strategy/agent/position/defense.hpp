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
#include "rj_strategy/agent/position_utils.hpp"
#include "rj_strategy/coordinator/marking.hpp"
#include "rj_strategy/coordinator/waller.hpp"

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

    std::string get_state_name() const override {
        return std::string(state_to_name(current_state_));
    }

    void die() override;
    void revive() override;

private:
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
        STEALING,          // Fighting for a loose ball as the closest robot
        SHOOTING,          // Shoot the ball on goal (only reachable from STEALING)
    };

    State update_state();
    State current_state_ = JOINING_WALL;
    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    static constexpr std::string_view state_to_name(State s) {
        switch (s) {
            case IDLING:
                return "IDLING";
            case JOINING_WALL:
                return "JOINING_WALL";
            case WALLING:
                return "WALLING";
            case SEARCHING:
                return "SEARCHING";
            case RECEIVING:
                return "RECEIVING";
            case PASSING:
                return "PASSING";
            case FACING:
                return "FACING";
            case MARKING:
                return "ENTERING_MARKING";
            case ENTERING_MARKING:
                return "ENTERING_MARKING";
            case STEALING:
                return "STEALING";
            case SHOOTING:
                return "SHOOTING";
        }
    }

    bool sent_join_marking_group_request_ = false;
    RJ::Time request_time_;

    bool pending_marking_state_ = false;

    /**
     * @brief Check if this agent could easily steal the ball (i.e. it is the
     * closest of our robots to a legally-accessible ball). Mirrors the stealing
     * check used by the Offense position.
     */
    bool can_steal_ball() const;
};

}  // namespace strategy
