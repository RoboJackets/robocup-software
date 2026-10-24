#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>

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

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination) {
        destination = rj_msgs::msg::AgentRequest{};
        if (const auto* join_wall_request =
                std::get_if<strategy::communication::JoinWallRequest>(&source)) {
            rj_msgs::msg::JoinWallRequest converted;
            rclcpp::TypeAdapter<strategy::communication::JoinWallRequest,
                                rj_msgs::msg::JoinWallRequest>::convert_to_ros_message(
                *join_wall_request, converted);
            destination.join_wall_request.emplace_back(converted);
        } else if (const auto* test_request =
                       std::get_if<strategy::communication::TestRequest>(&source)) {
            rj_msgs::msg::TestRequest converted;
            rclcpp::TypeAdapter<strategy::communication::TestRequest, rj_msgs::msg::TestRequest>::
                convert_to_ros_message(*test_request, converted);
            destination.test_request.emplace_back(converted);
        } else if (const auto* pass_request =
                       std::get_if<strategy::communication::PassRequest>(&source)) {
            rj_msgs::msg::PassRequest converted;
            rclcpp::TypeAdapter<strategy::communication::PassRequest, rj_msgs::msg::PassRequest>::
                convert_to_ros_message(*pass_request, converted);
            destination.pass_request.emplace_back(converted);
        } else if (const auto* scorer_request =
                       std::get_if<strategy::communication::ScorerRequest>(&source)) {
            rj_msgs::msg::ScorerRequest converted;
            rclcpp::TypeAdapter<strategy::communication::ScorerRequest,
                                rj_msgs::msg::ScorerRequest>::convert_to_ros_message(
                *scorer_request, converted);
            destination.scorer_request.emplace_back(converted);
        } else if (const auto* ball_in_transit_request =
                       std::get_if<strategy::communication::BallInTransitRequest>(&source)) {
            rj_msgs::msg::BallInTransitRequest converted;
            rclcpp::TypeAdapter<strategy::communication::BallInTransitRequest,
                                rj_msgs::msg::BallInTransitRequest>::convert_to_ros_message(
                *ball_in_transit_request, converted);
            destination.ball_in_transit_request.emplace_back(converted);
        } else if (const auto* seeker_request =
                       std::get_if<strategy::communication::SeekerRequest>(&source)) {
            rj_msgs::msg::SeekerRequest converted;
            rclcpp::TypeAdapter<strategy::communication::SeekerRequest,
                                rj_msgs::msg::SeekerRequest>::convert_to_ros_message(
                *seeker_request, converted);
            destination.seeker_request.emplace_back(converted);
        } else if (const auto* position_request =
                       std::get_if<strategy::communication::PositionRequest>(&source)) {
            rj_msgs::msg::PositionRequest converted;
            rclcpp::TypeAdapter<strategy::communication::PositionRequest,
                                rj_msgs::msg::PositionRequest>::convert_to_ros_message(
                *position_request, converted);
            destination.position_request.emplace_back(converted);
        } else if (const auto* leave_wall_request =
                       std::get_if<strategy::communication::LeaveWallRequest>(&source)) {
            rj_msgs::msg::LeaveWallRequest converted;
            rclcpp::TypeAdapter<strategy::communication::LeaveWallRequest,
                                rj_msgs::msg::LeaveWallRequest>::convert_to_ros_message(
                *leave_wall_request, converted);
            destination.leave_wall_request.emplace_back(converted);
        } else if (const auto* reset_scorer_request =
                       std::get_if<strategy::communication::ResetScorerRequest>(&source)) {
            rj_msgs::msg::ResetScorerRequest converted;
            rclcpp::TypeAdapter<strategy::communication::ResetScorerRequest,
                                rj_msgs::msg::ResetScorerRequest>::convert_to_ros_message(
                *reset_scorer_request, converted);
            destination.reset_scorer_request.emplace_back(converted);
        } else if (const auto* incoming_ball_request =
                       std::get_if<strategy::communication::IncomingBallRequest>(&source)) {
            rj_msgs::msg::IncomingBallRequest converted;
            rclcpp::TypeAdapter<strategy::communication::IncomingBallRequest,
                                rj_msgs::msg::IncomingBallRequest>::convert_to_ros_message(
                *incoming_ball_request, converted);
            destination.incoming_ball_request.emplace_back(converted);
        } else if (const auto* pass_received_request =
                       std::get_if<strategy::communication::PassReceivedRequest>(&source)) {
            rj_msgs::msg::PassReceivedRequest converted;
            rclcpp::TypeAdapter<strategy::communication::PassReceivedRequest,
                                rj_msgs::msg::PassReceivedRequest>::convert_to_ros_message(
                *pass_received_request, converted);
            destination.pass_received_request.emplace_back(converted);
        } else {
            throw std::runtime_error("Invalid variant of AgentRequest");
        }
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        strategy::communication::AgentRequest result;
        if (!source.join_wall_request.empty()) {
            strategy::communication::JoinWallRequest converted;
            rclcpp::TypeAdapter<strategy::communication::JoinWallRequest,
                                rj_msgs::msg::JoinWallRequest>::convert_to_custom(
                source.join_wall_request.front(), converted);
            result = converted;
        } else if (!source.test_request.empty()) {
            strategy::communication::TestRequest converted;
            rclcpp::TypeAdapter<strategy::communication::TestRequest, rj_msgs::msg::TestRequest>::
                convert_to_custom(source.test_request.front(), converted);
            result = converted;
        } else if (!source.pass_request.empty()) {
            strategy::communication::PassRequest converted;
            rclcpp::TypeAdapter<strategy::communication::PassRequest, rj_msgs::msg::PassRequest>::
                convert_to_custom(source.pass_request.front(), converted);
            result = converted;
        } else if (!source.scorer_request.empty()) {
            strategy::communication::ScorerRequest converted;
            rclcpp::TypeAdapter<strategy::communication::ScorerRequest,
                                rj_msgs::msg::ScorerRequest>::convert_to_custom(
                source.scorer_request.front(), converted);
            result = converted;
        } else if (!source.ball_in_transit_request.empty()) {
            strategy::communication::BallInTransitRequest converted;
            rclcpp::TypeAdapter<strategy::communication::BallInTransitRequest,
                                rj_msgs::msg::BallInTransitRequest>::convert_to_custom(
                source.ball_in_transit_request.front(), converted);
            result = converted;
        } else if (!source.seeker_request.empty()) {
            strategy::communication::SeekerRequest converted;
            rclcpp::TypeAdapter<strategy::communication::SeekerRequest,
                                rj_msgs::msg::SeekerRequest>::convert_to_custom(
                source.seeker_request.front(), converted);
            result = converted;
        } else if (!source.position_request.empty()) {
            strategy::communication::PositionRequest converted;
            rclcpp::TypeAdapter<strategy::communication::PositionRequest,
                                rj_msgs::msg::PositionRequest>::convert_to_custom(
                source.position_request.front(), converted);
            result = converted;
        } else if (!source.leave_wall_request.empty()) {
            strategy::communication::LeaveWallRequest converted;
            rclcpp::TypeAdapter<strategy::communication::LeaveWallRequest,
                                rj_msgs::msg::LeaveWallRequest>::convert_to_custom(
                source.leave_wall_request.front(), converted);
            result = converted;
        } else if (!source.reset_scorer_request.empty()) {
            strategy::communication::ResetScorerRequest converted;
            rclcpp::TypeAdapter<strategy::communication::ResetScorerRequest,
                                rj_msgs::msg::ResetScorerRequest>::convert_to_custom(
                source.reset_scorer_request.front(), converted);
            result = converted;
        } else if (!source.incoming_ball_request.empty()) {
            strategy::communication::IncomingBallRequest converted;
            rclcpp::TypeAdapter<strategy::communication::IncomingBallRequest,
                                rj_msgs::msg::IncomingBallRequest>::convert_to_custom(
                source.incoming_ball_request.front(), converted);
            result = converted;
        } else if (!source.pass_received_request.empty()) {
            strategy::communication::PassReceivedRequest converted;
            rclcpp::TypeAdapter<strategy::communication::PassReceivedRequest,
                                rj_msgs::msg::PassReceivedRequest>::convert_to_custom(
                source.pass_received_request.front(), converted);
            result = converted;
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

    static void convert_to_ros_message(const custom_type& source, ros_message_type& destination) {
        destination = ros_message_type{};
        rclcpp::TypeAdapter<strategy::communication::AgentRequest, rj_msgs::msg::AgentRequest>::
            convert_to_ros_message(source.associated_request, destination.associated_request);
        if (const auto* scorer_response =
                std::get_if<strategy::communication::ScorerResponse>(&(source.response))) {
            rj_msgs::msg::ScorerResponse converted;
            rclcpp::TypeAdapter<strategy::communication::ScorerResponse,
                                rj_msgs::msg::ScorerResponse>::convert_to_ros_message(
                *scorer_response, converted);
            destination.response.scorer_response.emplace_back(converted);
        } else if (const auto* leave_wall_response =
                       std::get_if<strategy::communication::LeaveWallResponse>(
                           &(source.response))) {
            rj_msgs::msg::LeaveWallResponse converted;
            rclcpp::TypeAdapter<strategy::communication::LeaveWallResponse,
                                rj_msgs::msg::LeaveWallResponse>::convert_to_ros_message(
                *leave_wall_response, converted);
            destination.response.leave_wall_response.emplace_back(converted);
        } else if (const auto* position_response =
                       std::get_if<strategy::communication::PositionResponse>(&(source.response))) {
            rj_msgs::msg::PositionResponse converted;
            rclcpp::TypeAdapter<strategy::communication::PositionResponse,
                                rj_msgs::msg::PositionResponse>::convert_to_ros_message(
                *position_response, converted);
            destination.response.position_response.emplace_back(converted);
        } else if (const auto* test_response =
                       std::get_if<strategy::communication::TestResponse>(&(source.response))) {
            rj_msgs::msg::TestResponse converted;
            rclcpp::TypeAdapter<strategy::communication::TestResponse, rj_msgs::msg::TestResponse>::
                convert_to_ros_message(*test_response, converted);
            destination.response.test_response.emplace_back(converted);
        } else if (const auto* pass_response =
                       std::get_if<strategy::communication::PassResponse>(&(source.response))) {
            rj_msgs::msg::PassResponse converted;
            rclcpp::TypeAdapter<strategy::communication::PassResponse, rj_msgs::msg::PassResponse>::
                convert_to_ros_message(*pass_response, converted);
            destination.response.pass_response.emplace_back(converted);
        } else if (const auto* acknowledge =
                       std::get_if<strategy::communication::Acknowledge>(&(source.response))) {
            rj_msgs::msg::Acknowledge converted;
            rclcpp::TypeAdapter<strategy::communication::Acknowledge, rj_msgs::msg::Acknowledge>::
                convert_to_ros_message(*acknowledge, converted);
            destination.response.acknowledge.emplace_back(converted);
        } else if (const auto* join_wall_response =
                       std::get_if<strategy::communication::JoinWallResponse>(&(source.response))) {
            rj_msgs::msg::JoinWallResponse converted;
            rclcpp::TypeAdapter<strategy::communication::JoinWallResponse,
                                rj_msgs::msg::JoinWallResponse>::convert_to_ros_message(
                *join_wall_response, converted);
            destination.response.join_wall_response.emplace_back(converted);
        } else {
            throw std::runtime_error("Invalid variant of AgentResponse");
        }
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        strategy::communication::AgentResponse result;
        rclcpp::TypeAdapter<strategy::communication::AgentRequest, rj_msgs::msg::AgentRequest>::
            convert_to_custom(source.associated_request, result.associated_request);
        if (!source.response.scorer_response.empty()) {
            strategy::communication::ScorerResponse converted;
            rclcpp::TypeAdapter<strategy::communication::ScorerResponse,
                                rj_msgs::msg::ScorerResponse>::convert_to_custom(
                source.response.scorer_response.front(), converted);
            result.response = converted;
        } else if (!source.response.leave_wall_response.empty()) {
            strategy::communication::LeaveWallResponse converted;
            rclcpp::TypeAdapter<strategy::communication::LeaveWallResponse,
                                rj_msgs::msg::LeaveWallResponse>::convert_to_custom(
                source.response.leave_wall_response.front(), converted);
            result.response = converted;
        } else if (!source.response.position_response.empty()) {
            strategy::communication::PositionResponse converted;
            rclcpp::TypeAdapter<strategy::communication::PositionResponse,
                                rj_msgs::msg::PositionResponse>::convert_to_custom(
                source.response.position_response.front(), converted);
            result.response = converted;
        } else if (!source.response.test_response.empty()) {
            strategy::communication::TestResponse converted;
            rclcpp::TypeAdapter<strategy::communication::TestResponse, rj_msgs::msg::TestResponse>::
                convert_to_custom(source.response.test_response.front(), converted);
            result.response = converted;
        } else if (!source.response.pass_response.empty()) {
            strategy::communication::PassResponse converted;
            rclcpp::TypeAdapter<strategy::communication::PassResponse, rj_msgs::msg::PassResponse>::
                convert_to_custom(source.response.pass_response.front(), converted);
            result.response = converted;
        } else if (!source.response.acknowledge.empty()) {
            strategy::communication::Acknowledge converted;
            rclcpp::TypeAdapter<strategy::communication::Acknowledge, rj_msgs::msg::Acknowledge>::
                convert_to_custom(source.response.acknowledge.front(), converted);
            result.response = converted;
        } else if (!source.response.join_wall_response.empty()) {
            strategy::communication::JoinWallResponse converted;
            rclcpp::TypeAdapter<strategy::communication::JoinWallResponse,
                                rj_msgs::msg::JoinWallResponse>::convert_to_custom(
                source.response.join_wall_response.front(), converted);
            result.response = converted;
        } else {
            throw std::runtime_error("Invalid variant of AgentResponse");
        }
        destination = result;
    }
};

}  // namespace rclcpp