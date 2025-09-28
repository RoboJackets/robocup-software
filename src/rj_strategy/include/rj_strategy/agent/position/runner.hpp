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

class Runner : public Position {
    public:
        Runner(int r_id);
        ~Runner() override = default;
        Runner(const Position& other);

    //     void receive_communication_response(communication::AgentPosResponseWrapper response) override;
    // communication::PosAgentResponseWrapper receive_communication_request(
    //     communication::AgentPosRequestWrapper request) override;

    // void derived_acknowledge_pass() override;
    // void derived_pass_ball() override;
    // void derived_acknowledge_ball_in_transit() override;
    std::string get_current_state() override;

    void die() override;
    void revive() override;
    
    private:
        std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

        enum State {
            RIGHT_SIDE, // travelling along the right side of the rectangle
            LEFT_SIDE, // travelling along the left side of the rectangle
            TOP_SIDE, // travelling along the top side of the rectangle
            BOTTOM_SIDE, // travelling along the bottom side of the rectangle
        };

        State update_state();

        std::optional<RobotIntent> state_to_task(RobotIntent intent);

        State current_state_ = RIGHT_SIDE;

};
};