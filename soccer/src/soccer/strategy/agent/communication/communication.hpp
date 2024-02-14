#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>

#include "rj_msgs/msg/agent_request.hpp"
#include "rj_msgs/msg/agent_response.hpp"
#include "rj_msgs/msg/agent_response_variant.hpp"
#include "leave_wall_request.hpp"
#include "pass_request.hpp"
#include "test_request.hpp"
#include "ball_in_transit_request.hpp"
#include "kicker_request.hpp"
#include "join_wall_request.hpp"
#include "scorer_request.hpp"
#include "incoming_ball_request.hpp"
#include "reset_scorer_request.hpp"
#include "position_request.hpp"
#include "leave_wall_response.hpp"
#include "scorer_response.hpp"
#include "join_wall_response.hpp"
#include "acknowledge.hpp"
#include "position_response.hpp"
#include "pass_response.hpp"
#include "test_response.hpp"

namespace strategy::communication {

/**
* @brief a conglomeration of the different request types.
*/
using AgentRequest = std::variant<LeaveWallRequest, PassRequest, TestRequest, BallInTransitRequest, KickerRequest, JoinWallRequest, ScorerRequest, IncomingBallRequest, ResetScorerRequest, PositionRequest>;

/**
* @brief a conglomeration of the different response types.
*/
using AgentResponseVariant = std::variant<LeaveWallResponse, ScorerResponse, JoinWallResponse, Acknowledge, PositionResponse, PassResponse, TestResponse>;

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

}

namespace rj_convert {

template <>
struct RosConverter<strategy::communication::AgentRequest, rj_msgs::msg::AgentRequest> {
	static rj_msgs::msg::AgentRequest to_ros(const strategy::communication::AgentRequest& from) {
		rj_msgs::msg::AgentRequest result;
		if (const auto* leave_wall_request = std::get_if<strategy::communication::LeaveWallRequest>(&from)) {
			result.leave_wall_request.emplace_back(convert_to_ros(*leave_wall_request));
		} else if (const auto* pass_request = std::get_if<strategy::communication::PassRequest>(&from)) {
			result.pass_request.emplace_back(convert_to_ros(*pass_request));
		} else if (const auto* test_request = std::get_if<strategy::communication::TestRequest>(&from)) {
			result.test_request.emplace_back(convert_to_ros(*test_request));
		} else if (const auto* ball_in_transit_request = std::get_if<strategy::communication::BallInTransitRequest>(&from)) {
			result.ball_in_transit_request.emplace_back(convert_to_ros(*ball_in_transit_request));
		} else if (const auto* kicker_request = std::get_if<strategy::communication::KickerRequest>(&from)) {
			result.kicker_request.emplace_back(convert_to_ros(*kicker_request));
		} else if (const auto* join_wall_request = std::get_if<strategy::communication::JoinWallRequest>(&from)) {
			result.join_wall_request.emplace_back(convert_to_ros(*join_wall_request));
		} else if (const auto* scorer_request = std::get_if<strategy::communication::ScorerRequest>(&from)) {
			result.scorer_request.emplace_back(convert_to_ros(*scorer_request));
		} else if (const auto* incoming_ball_request = std::get_if<strategy::communication::IncomingBallRequest>(&from)) {
			result.incoming_ball_request.emplace_back(convert_to_ros(*incoming_ball_request));
		} else if (const auto* reset_scorer_request = std::get_if<strategy::communication::ResetScorerRequest>(&from)) {
			result.reset_scorer_request.emplace_back(convert_to_ros(*reset_scorer_request));
		} else if (const auto* position_request = std::get_if<strategy::communication::PositionRequest>(&from)) {
			result.position_request.emplace_back(convert_to_ros(*position_request));
		} else {
			throw std::runtime_error("Invalid variant of AgentRequest");
		}
		return result;
	}

	static strategy::communication::AgentRequest from_ros(const rj_msgs::msg::AgentRequest& from) {
		strategy::communication::AgentRequest result;
		if (!from.leave_wall_request.empty()) {
			result = convert_from_ros(from.leave_wall_request.front());
		} else if (!from.pass_request.empty()) {
			result = convert_from_ros(from.pass_request.front());
		} else if (!from.test_request.empty()) {
			result = convert_from_ros(from.test_request.front());
		} else if (!from.ball_in_transit_request.empty()) {
			result = convert_from_ros(from.ball_in_transit_request.front());
		} else if (!from.kicker_request.empty()) {
			result = convert_from_ros(from.kicker_request.front());
		} else if (!from.join_wall_request.empty()) {
			result = convert_from_ros(from.join_wall_request.front());
		} else if (!from.scorer_request.empty()) {
			result = convert_from_ros(from.scorer_request.front());
		} else if (!from.incoming_ball_request.empty()) {
			result = convert_from_ros(from.incoming_ball_request.front());
		} else if (!from.reset_scorer_request.empty()) {
			result = convert_from_ros(from.reset_scorer_request.front());
		} else if (!from.position_request.empty()) {
			result = convert_from_ros(from.position_request.front());
		} else {
			throw std::runtime_error("Invalid variant of AgentRequest");
		}
		return result;
	}

};

ASSOCIATE_CPP_ROS(strategy::communication::AgentRequest, rj_msgs::msg::AgentRequest);

template <>
struct RosConverter<strategy::communication::AgentResponse, rj_msgs::msg::AgentResponse> {
	static rj_msgs::msg::AgentResponse to_ros(const strategy::communication::AgentResponse& from) {
		rj_msgs::msg::AgentResponse result;
		result.associated_request = convert_to_ros(from.associated_request);
		if (const auto* leave_wall_response = std::get_if<strategy::communication::LeaveWallResponse>(&(from.response))) {
			result.response.leave_wall_response.emplace_back(convert_to_ros(*leave_wall_response));
		} else if (const auto* scorer_response = std::get_if<strategy::communication::ScorerResponse>(&(from.response))) {
			result.response.scorer_response.emplace_back(convert_to_ros(*scorer_response));
		} else if (const auto* join_wall_response = std::get_if<strategy::communication::JoinWallResponse>(&(from.response))) {
			result.response.join_wall_response.emplace_back(convert_to_ros(*join_wall_response));
		} else if (const auto* acknowledge = std::get_if<strategy::communication::Acknowledge>(&(from.response))) {
			result.response.acknowledge.emplace_back(convert_to_ros(*acknowledge));
		} else if (const auto* position_response = std::get_if<strategy::communication::PositionResponse>(&(from.response))) {
			result.response.position_response.emplace_back(convert_to_ros(*position_response));
		} else if (const auto* pass_response = std::get_if<strategy::communication::PassResponse>(&(from.response))) {
			result.response.pass_response.emplace_back(convert_to_ros(*pass_response));
		} else if (const auto* test_response = std::get_if<strategy::communication::TestResponse>(&(from.response))) {
			result.response.test_response.emplace_back(convert_to_ros(*test_response));
		} else {
			throw std::runtime_error("Invalid variant of AgentResponse");
		}
		return result;
	}

	static strategy::communication::AgentResponse from_ros(const rj_msgs::msg::AgentResponse& from) {
		strategy::communication::AgentResponse result;
		result.associated_request = convert_from_ros(from.associated_request);
		if (!from.response.leave_wall_response.empty()) {
			result.response = convert_from_ros(from.response.leave_wall_response.front());
		} else if (!from.response.scorer_response.empty()) {
			result.response = convert_from_ros(from.response.scorer_response.front());
		} else if (!from.response.join_wall_response.empty()) {
			result.response = convert_from_ros(from.response.join_wall_response.front());
		} else if (!from.response.acknowledge.empty()) {
			result.response = convert_from_ros(from.response.acknowledge.front());
		} else if (!from.response.position_response.empty()) {
			result.response = convert_from_ros(from.response.position_response.front());
		} else if (!from.response.pass_response.empty()) {
			result.response = convert_from_ros(from.response.pass_response.front());
		} else if (!from.response.test_response.empty()) {
			result.response = convert_from_ros(from.response.test_response.front());
		} else {
			throw std::runtime_error("Invalid variant of AgentResponse");
		}
		return result;
	}

};

ASSOCIATE_CPP_ROS(strategy::communication::AgentResponse, rj_msgs::msg::AgentResponse);

}