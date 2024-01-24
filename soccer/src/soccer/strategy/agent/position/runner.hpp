#pragma once

#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/action/robot_move.hpp>

#include "planning/instant.hpp"
#include "position.hpp"
#include "rj_common/time.hpp"
#include "rj_geometry/geometry_conversions.hpp"
#include "rj_geometry/point.hpp"

namespace strategy {

class Runner : public Position {
public:
    Runner(int r_id);
    ~Runner() override = default;

    void receive_communication_response(communication::AgentPosResponseWrapper response){};
    communication::PosAgentResponseWrapper receive_communication_request(
        communication::AgentPosRequestWrapper request){};

    void derived_acknowledge_pass(){};
    void derived_pass_ball(){};
    void derived_acknowledge_ball_in_transit(){};

private:

    bool kicking_{true};

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
    
    enum State {
        RUNFIRST,
        RUNSECOND,
        RUNTHIRD,
        RUNFOURTH,
    };

    State update_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    State current_state_ = RUNFIRST;

    bool scorer_ = false;
    bool last_scorer_ = false;

    void send_scorer_request(){};

    communication::ScorerResponse receive_scorer_request(
        communication::ScorerRequest scorer_request){}

    void handle_scorer_response(
        const std::vector<communication::AgentResponseVariant>& scorer_responses){};
    
    
    void send_reset_scorer_request(){};

    communication::Acknowledge receive_reset_scorer_request(){};

};

} //namespace strategy

