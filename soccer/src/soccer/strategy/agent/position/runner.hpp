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

    void receive_communication_response(communication::AgentPosResponseWrapper response) override;
    communication::PosAgentResponseWrapper receive_communication_request(
        communication::AgentPosRequestWrapper request) override;

    void derived_acknowledge_pass() override;
    void derived_pass_ball() override;
    void derived_acknowledge_ball_in_transit() override;

private:
    bool kicking_{true};

    std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;
    // TODO (Kevin): strategy design pattern for BallHandler/Receiver

    enum State {
        IDLING,          // simply staying in place
        MOVING_LSIDE,
        MOVING_USIDE,
        MOVING_RSIDE,
        MOVING_DSIDE,
    };

    State update_state();

    std::optional<RobotIntent> state_to_task(RobotIntent intent);

    // current state of the offensive agent (state machine)
    State current_state_ = IDLING;
    bool at_corner = false;

    bool scorer_ = false;
    bool last_scorer_ = false;

    /**
     * @brief Send request to the other robots to see if this robot should be the scorer
     *
     */
    void send_scorer_request();

    /**
     * @brief Finds this robot's distance to the ball and sends it back to the robot who asked if
     * it should be the scorer
     *
     * @param scorer_request request from the prospective scorer robot
     * @return communication::ScorerResponse this robots response to the prospective scorer robot
     */
    communication::ScorerResponse receive_scorer_request(
        communication::ScorerRequest scorer_request);

    /**
     * @brief This agent can go through the distance of every other offensive robot from the goal
     * to decide whether this robot should become the scorer.
     *
     * @param scorer_responses a vector of the distance to the ball for each other offense robot
     */
    void handle_scorer_response(
        const std::vector<communication::AgentResponseVariant>& scorer_responses);

    /**
     * @brief Send a request to the other offensive agents to let them know to reset who is the
     * scorer.
     *
     */
    void send_reset_scorer_request();

    /**
     * @brief When the scorer is being reset all offense robots should send a scorer requests
     * and reset their last_scorer_ flag.
     *
     * @param reset_scorer_request the reset scorer request
     * @return communication::Acknowledge acknowledgement that this robot will reset their scorer
     * status
     */
    communication::Acknowledge receive_reset_scorer_request();
};

// public:
//     Runner(int r_id);
//     ~Runner() override = default;

//     // i don't know if i need these
//     void receive_communication_response(communication::AgentPosResponseWrapper response) override;
//     communication::PosAgentResponseWrapper receive_communication_request(
//         communication::AgentPosRequestWrapper request) override;

//     void derived_acknowledge_pass() override;
//     void derived_pass_ball() override;
//     void derived_acknowledge_ball_in_transit() override;

// private:
//     std::optional<RobotIntent> derived_get_task(RobotIntent intent) override;

//     // hardcoding is probably not the best idea
//     enum State {
//         MOVING_L,
//         MOVING_R,
//         MOVING_U,
//         MOVING_D,
//         TURNING_UL,
//         TURNING_UR,
//         TURNING_DR,
//         TURNING_DL,
//     };

//     State update_state();
    
//     std::optional<RobotIntent> state_to_task(RobotIntent intent);

//     State current_state_ = MOVING_L;
//     bool at_corner = false;

//     // garbage beyond here
//     bool scorer_ = false;
//     bool last_scorer_ = false;

//     /**
//      * @brief Send request to the other robots to see if this robot should be the scorer
//      *
//      */
//     void send_scorer_request();

//     /**
//      * @brief Finds this robot's distance to the ball and sends it back to the robot who asked if
//      * it should be the scorer
//      *
//      * @param scorer_request request from the prospective scorer robot
//      * @return communication::ScorerResponse this robots response to the prospective scorer robot
//      */
//     communication::ScorerResponse receive_scorer_request(
//         communication::ScorerRequest scorer_request);

//     /**
//      * @brief This agent can go through the distance of every other offensive robot from the goal
//      * to decide whether this robot should become the scorer.
//      *
//      * @param scorer_responses a vector of the distance to the ball for each other offense robot
//      */
//     void handle_scorer_response(
//         const std::vector<communication::AgentResponseVariant>& scorer_responses);

//     /**
//      * @brief Send a request to the other offensive agents to let them know to reset who is the
//      * scorer.
//      *
//      */
//     void send_reset_scorer_request();

//     /**
//      * @brief When the scorer is being reset all offense robots should send a scorer requests
//      * and reset their last_scorer_ flag.
//      *
//      * @param reset_scorer_request the reset scorer request
//      * @return communication::Acknowledge acknowledgement that this robot will reset their scorer
//      * status
//      */
//     communication::Acknowledge receive_reset_scorer_request();
// }; 

}