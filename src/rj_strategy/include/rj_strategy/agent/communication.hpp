#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_msgs/msg/agent_request.hpp>
#include <rj_msgs/msg/agent_response.hpp>
#include <rj_msgs/msg/agent_response_variant.hpp>

#include "rj_strategy/agent/communication/acknowledge.hpp"
#include "rj_strategy/agent/communication/ball_in_transit_request.hpp"
#include "rj_strategy/agent/communication/incoming_ball_request.hpp"
#include "rj_strategy/agent/communication/join_wall_request.hpp"
#include "rj_strategy/agent/communication/join_wall_response.hpp"
#include "rj_strategy/agent/communication/leave_wall_request.hpp"
#include "rj_strategy/agent/communication/leave_wall_response.hpp"
#include "rj_strategy/agent/communication/pass_received_request.hpp"
#include "rj_strategy/agent/communication/pass_request.hpp"
#include "rj_strategy/agent/communication/pass_response.hpp"
#include "rj_strategy/agent/communication/position_request.hpp"
#include "rj_strategy/agent/communication/position_response.hpp"
#include "rj_strategy/agent/communication/reset_scorer_request.hpp"
#include "rj_strategy/agent/communication/scorer_request.hpp"
#include "rj_strategy/agent/communication/scorer_response.hpp"
#include "rj_strategy/agent/communication/seeker_request.hpp"
#include "rj_strategy/agent/communication/test_request.hpp"
#include "rj_strategy/agent/communication/test_response.hpp"

namespace strategy::communication {

/**
 * @brief a conglomeration of the different request types.
 */
using AgentRequest =
    std::variant<JoinWallRequest, TestRequest, PassRequest, ScorerRequest, BallInTransitRequest,
                 SeekerRequest, PositionRequest, LeaveWallRequest, ResetScorerRequest,
                 IncomingBallRequest, PassReceivedRequest>;

/**
 * @brief a conglomeration of the different response types.
 */
using AgentResponseVariant =
    std::variant<ScorerResponse, LeaveWallResponse, PositionResponse, TestResponse, PassResponse,
                 Acknowledge, JoinWallResponse>;

/**
 * @brief response message that is sent from the receiver of the request to the
 * sender of the request with an accompanying response.
 *
 * The agent response is the actual thing that gets sent from a receiver back
 * to the sender.
 *
 */
struct AgentResponse {
    AgentRequest associated_request;
    AgentResponseVariant response;
};

bool operator==(const AgentResponse& a, const AgentResponse& b);

/**
 * @brief Wraps a communication request by giving the intended destination of the
 * communication.
 *
 * positions will create this and send it to their agent action client which will
 * send out the request according to their specifications.
 *
 */
struct PosAgentRequestWrapper {
    AgentRequest request;
    std::vector<u_int8_t> target_agents;
    bool broadcast;
    bool urgent;
};

/**
 * @brief Wraps a communication response to ensure symmetry for agent-to-agent
 * communication.
 *
 * this wrapper is placed on agent responses to promote symmetry across the request
 * response system to make understanding easier.  All this struct does is make explicit
 * that this response is going from the position to the agent.
 *
 */
struct PosAgentResponseWrapper {
    AgentResponseVariant response;
};

/**
 * @brief Wraps a communication request to ensure symmetry for agent-to-agent
 * communication.
 *
 * Like the PosAgentResponseWrapper, this struct does nothing other than make the request
 * response system more symmetrical and (hopefully) more easy to understand.  All this struct
 * does is make it explicit that this request is being passed from the agent to the agent to
 * the position.
 *
 */
struct AgentPosRequestWrapper {
    AgentRequest request;
};

/**
 * @brief Wraps a communication response by giving the robot the communication is from.
 *
 * the AgentPosResponseWrapper is the actual thing being passed from the agent to the position
 * once either the timeout period was reached or enough responses were received.  Ideally, the
 * contents of this wrapper should contain all of the non-message specific fields that a position
 * will need to handle a response.
 *
 */
struct AgentPosResponseWrapper {
    AgentRequest associated_request;
    std::vector<u_int8_t> to_robot_ids;
    std::vector<u_int8_t> received_robot_ids;
    bool broadcast;
    bool urgent;
    RJ::Time created;
    std::vector<AgentResponseVariant> responses;
};

}  // namespace strategy::communication

namespace rclcpp {

template <>
struct TypeAdapter<strategy::communication::AgentRequest, rj_msgs::msg::AgentRequest> {
    using is_specialized = std::true_type;
    using custom_type = strategy::communication::AgentRequest;
    using ros_message_type = rj_msgs::msg::AgentRequest;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination = rj_msgs::msg::AgentRequest{};
        if (const auto* join_wall_request =
                std::get_if<strategy::communication::JoinWallRequest>(&source)) {
            destination.join_wall_request.emplace_back(
                rj_convert::convert_to_ros<strategy::communication::JoinWallRequest,
                                           rj_msgs::msg::JoinWallRequest>(*join_wall_request));
        } else if (const auto* test_request =
                       std::get_if<strategy::communication::TestRequest>(&source)) {
            destination.test_request.emplace_back(
                rj_convert::convert_to_ros<strategy::communication::TestRequest,
                                           rj_msgs::msg::TestRequest>(*test_request));
        } else if (const auto* pass_request =
                       std::get_if<strategy::communication::PassRequest>(&source)) {
            destination.pass_request.emplace_back(
                rj_convert::convert_to_ros<strategy::communication::PassRequest,
                                           rj_msgs::msg::PassRequest>(*pass_request));
        } else if (const auto* scorer_request =
                       std::get_if<strategy::communication::ScorerRequest>(&source)) {
            destination.scorer_request.emplace_back(
                rj_convert::convert_to_ros<strategy::communication::ScorerRequest,
                                           rj_msgs::msg::ScorerRequest>(*scorer_request));
        } else if (const auto* ball_in_transit_request =
                       std::get_if<strategy::communication::BallInTransitRequest>(&source)) {
            destination.ball_in_transit_request.emplace_back(
                rj_convert::convert_to_ros<strategy::communication::BallInTransitRequest,
                                           rj_msgs::msg::BallInTransitRequest>(
                    *ball_in_transit_request));
        } else if (const auto* seeker_request =
                       std::get_if<strategy::communication::SeekerRequest>(&source)) {
            destination.seeker_request.emplace_back(
                rj_convert::convert_to_ros<strategy::communication::SeekerRequest,
                                           rj_msgs::msg::SeekerRequest>(*seeker_request));
        } else if (const auto* position_request =
                       std::get_if<strategy::communication::PositionRequest>(&source)) {
            destination.position_request.emplace_back(
                rj_convert::convert_to_ros<strategy::communication::PositionRequest,
                                           rj_msgs::msg::PositionRequest>(*position_request));
        } else if (const auto* leave_wall_request =
                       std::get_if<strategy::communication::LeaveWallRequest>(&source)) {
            destination.leave_wall_request.emplace_back(
                rj_convert::convert_to_ros<strategy::communication::LeaveWallRequest,
                                           rj_msgs::msg::LeaveWallRequest>(*leave_wall_request));
        } else if (const auto* reset_scorer_request =
                       std::get_if<strategy::communication::ResetScorerRequest>(&source)) {
            destination.reset_scorer_request.emplace_back(
                rj_convert::convert_to_ros<strategy::communication::ResetScorerRequest,
                                           rj_msgs::msg::ResetScorerRequest>(*reset_scorer_request));
        } else if (const auto* incoming_ball_request =
                       std::get_if<strategy::communication::IncomingBallRequest>(&source)) {
            destination.incoming_ball_request.emplace_back(
                rj_convert::convert_to_ros<strategy::communication::IncomingBallRequest,
                                           rj_msgs::msg::IncomingBallRequest>(
                    *incoming_ball_request));
        } else if (const auto* pass_received_request =
                       std::get_if<strategy::communication::PassReceivedRequest>(&source)) {
            destination.pass_received_request.emplace_back(
                rj_convert::convert_to_ros<strategy::communication::PassReceivedRequest,
                                           rj_msgs::msg::PassReceivedRequest>(*pass_received_request));
        } else {
            throw std::runtime_error("Invalid variant of AgentRequest");
        }
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        strategy::communication::AgentRequest result;
        if (!source.join_wall_request.empty()) {
            result = rj_convert::convert_from_ros<rj_msgs::msg::JoinWallRequest,
                                                  strategy::communication::JoinWallRequest>(
                source.join_wall_request.front());
        } else if (!source.test_request.empty()) {
            result = rj_convert::convert_from_ros<rj_msgs::msg::TestRequest,
                                                  strategy::communication::TestRequest>(
                source.test_request.front());
        } else if (!source.pass_request.empty()) {
            result = rj_convert::convert_from_ros<rj_msgs::msg::PassRequest,
                                                  strategy::communication::PassRequest>(
                source.pass_request.front());
        } else if (!source.scorer_request.empty()) {
            result = rj_convert::convert_from_ros<rj_msgs::msg::ScorerRequest,
                                                  strategy::communication::ScorerRequest>(
                source.scorer_request.front());
        } else if (!source.ball_in_transit_request.empty()) {
            result = rj_convert::convert_from_ros<rj_msgs::msg::BallInTransitRequest,
                                                  strategy::communication::BallInTransitRequest>(
                source.ball_in_transit_request.front());
        } else if (!source.seeker_request.empty()) {
            result = rj_convert::convert_from_ros<rj_msgs::msg::SeekerRequest,
                                                  strategy::communication::SeekerRequest>(
                source.seeker_request.front());
        } else if (!source.position_request.empty()) {
            result = rj_convert::convert_from_ros<rj_msgs::msg::PositionRequest,
                                                  strategy::communication::PositionRequest>(
                source.position_request.front());
        } else if (!source.leave_wall_request.empty()) {
            result = rj_convert::convert_from_ros<rj_msgs::msg::LeaveWallRequest,
                                                  strategy::communication::LeaveWallRequest>(
                source.leave_wall_request.front());
        } else if (!source.reset_scorer_request.empty()) {
            result = rj_convert::convert_from_ros<rj_msgs::msg::ResetScorerRequest,
                                                  strategy::communication::ResetScorerRequest>(
                source.reset_scorer_request.front());
        } else if (!source.incoming_ball_request.empty()) {
            result = rj_convert::convert_from_ros<rj_msgs::msg::IncomingBallRequest,
                                                  strategy::communication::IncomingBallRequest>(
                source.incoming_ball_request.front());
        } else if (!source.pass_received_request.empty()) {
            result = rj_convert::convert_from_ros<rj_msgs::msg::PassReceivedRequest,
                                                  strategy::communication::PassReceivedRequest>(
                source.pass_received_request.front());
        } else {
            throw std::runtime_error("Invalid variant of AgentRequest");
        }
        destination = result;
    }
};


template <>
struct TypeAdapter<strategy::communication::AgentResponse, rj_msgs::msg::AgentResponse> {
    using is_specialized = std::true_type;
    using custom_type = strategy::communication::AgentResponse;
    using ros_message_type = rj_msgs::msg::AgentResponse;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination = ros_message_type{};
        destination.associated_request =
            rj_convert::convert_to_ros<strategy::communication::AgentRequest,
                                       rj_msgs::msg::AgentRequest>(source.associated_request);
        if (const auto* scorer_response =
                std::get_if<strategy::communication::ScorerResponse>(&(source.response))) {
            destination.response.scorer_response.emplace_back(
                rj_convert::convert_to_ros<strategy::communication::ScorerResponse,
                                           rj_msgs::msg::ScorerResponse>(*scorer_response));
        } else if (const auto* leave_wall_response =
                       std::get_if<strategy::communication::LeaveWallResponse>(&(source.response))) {
            destination.response.leave_wall_response.emplace_back(
                rj_convert::convert_to_ros<strategy::communication::LeaveWallResponse,
                                           rj_msgs::msg::LeaveWallResponse>(*leave_wall_response));
        } else if (const auto* position_response =
                       std::get_if<strategy::communication::PositionResponse>(&(source.response))) {
            destination.response.position_response.emplace_back(
                rj_convert::convert_to_ros<strategy::communication::PositionResponse,
                                           rj_msgs::msg::PositionResponse>(*position_response));
        } else if (const auto* test_response =
                       std::get_if<strategy::communication::TestResponse>(&(source.response))) {
            destination.response.test_response.emplace_back(
                rj_convert::convert_to_ros<strategy::communication::TestResponse,
                                           rj_msgs::msg::TestResponse>(*test_response));
        } else if (const auto* pass_response =
                       std::get_if<strategy::communication::PassResponse>(&(source.response))) {
            destination.response.pass_response.emplace_back(
                rj_convert::convert_to_ros<strategy::communication::PassResponse,
                                           rj_msgs::msg::PassResponse>(*pass_response));
        } else if (const auto* acknowledge =
                       std::get_if<strategy::communication::Acknowledge>(&(source.response))) {
            destination.response.acknowledge.emplace_back(
                rj_convert::convert_to_ros<strategy::communication::Acknowledge,
                                           rj_msgs::msg::Acknowledge>(*acknowledge));
        } else if (const auto* join_wall_response =
                       std::get_if<strategy::communication::JoinWallResponse>(&(source.response))) {
            destination.response.join_wall_response.emplace_back(
                rj_convert::convert_to_ros<strategy::communication::JoinWallResponse,
                                           rj_msgs::msg::JoinWallResponse>(*join_wall_response));
        } else {
            throw std::runtime_error("Invalid variant of AgentResponse");
        }
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        strategy::communication::AgentResponse result;
        result.associated_request =
            rj_convert::convert_from_ros<rj_msgs::msg::AgentRequest,
                                         strategy::communication::AgentRequest>(
                source.associated_request);
        if (!source.response.scorer_response.empty()) {
            result.response =
                rj_convert::convert_from_ros<rj_msgs::msg::ScorerResponse,
                                             strategy::communication::ScorerResponse>(
                    source.response.scorer_response.front());
        } else if (!source.response.leave_wall_response.empty()) {
            result.response =
                rj_convert::convert_from_ros<rj_msgs::msg::LeaveWallResponse,
                                             strategy::communication::LeaveWallResponse>(
                    source.response.leave_wall_response.front());
        } else if (!source.response.position_response.empty()) {
            result.response =
                rj_convert::convert_from_ros<rj_msgs::msg::PositionResponse,
                                             strategy::communication::PositionResponse>(
                    source.response.position_response.front());
        } else if (!source.response.test_response.empty()) {
            result.response =
                rj_convert::convert_from_ros<rj_msgs::msg::TestResponse,
                                             strategy::communication::TestResponse>(
                    source.response.test_response.front());
        } else if (!source.response.pass_response.empty()) {
            result.response =
                rj_convert::convert_from_ros<rj_msgs::msg::PassResponse,
                                             strategy::communication::PassResponse>(
                    source.response.pass_response.front());
        } else if (!source.response.acknowledge.empty()) {
            result.response =
                rj_convert::convert_from_ros<rj_msgs::msg::Acknowledge,
                                             strategy::communication::Acknowledge>(
                    source.response.acknowledge.front());
        } else if (!source.response.join_wall_response.empty()) {
            result.response =
                rj_convert::convert_from_ros<rj_msgs::msg::JoinWallResponse,
                                             strategy::communication::JoinWallResponse>(
                    source.response.join_wall_response.front());
        } else {
            throw std::runtime_error("Invalid variant of AgentResponse");
        }
        destination = result;
    }
};


}  // namespace rclcpp