#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>

#include "rj_msgs/msg/acknowledge.hpp"
#include "rj_msgs/msg/agent_request.hpp"
#include "rj_msgs/msg/agent_response.hpp"
#include "rj_msgs/msg/agent_response_variant.hpp"
#include "rj_msgs/msg/incoming_ball_request.hpp"
#include "rj_msgs/msg/join_wall_request.hpp"
#include "rj_msgs/msg/join_wall_response.hpp"
#include "rj_msgs/msg/leave_wall_request.hpp"
#include "rj_msgs/msg/leave_wall_response.hpp"
#include "rj_msgs/msg/pass_request.hpp"
#include "rj_msgs/msg/pass_response.hpp"
#include "rj_msgs/msg/position_request.hpp"
#include "rj_msgs/msg/position_response.hpp"
#include "rj_msgs/msg/reset_scorer_request.hpp"
#include "rj_msgs/msg/scorer_request.hpp"
#include "rj_msgs/msg/scorer_response.hpp"
#include "rj_msgs/msg/test_request.hpp"
#include "rj_msgs/msg/test_response.hpp"

namespace strategy::communication {

// BEGIN REQUEST TYPES //

/**
 * @brief request sent by an agent to one or many agents indicating
 * that it would like to pass the bass.
 *
 * pass requests exist so that agents can find out which other agents
 * are open for passes.
 *
 */
struct PassRequest {
    u_int32_t request_uid;
    bool direct;
    u_int8_t from_robot_id;
};
bool operator==(const PassRequest& a, const PassRequest& b);

/**
 * @brief request sent by an agent to one or many agents indicating
 * that it would like to know the current position the agent is playing.
 *
 * position requests were originally used for testing, however they may be
 * useful for coordinating complex behaviors.  For example, agent a may need
 * to know that agent b is currently a waller before attempting to ask agent b
 * for help.
 */
struct PositionRequest {
    u_int32_t request_uid;
};
bool operator==(const PositionRequest& a, const PositionRequest& b);

/**
 * @brief request sent by an agent to one or many target agents that
 * can be used for testing communication between different agents.
 *
 * the test request was used mainly for testing that messages were being successfully
 * sent between the various agents.
 *
 */
struct TestRequest {
    u_int32_t request_uid;
};
bool operator==(const TestRequest& a, const TestRequest& b);

/**
 * @brief request sent by an agent to a specific agent indicating that
 * this agent is going to kick the ball to the other agent iff the other
 * agent acknowledges this message.
 *
 * agents will send an incoming ball request to the agent they will be passing
 * to.
 *
 */
struct IncomingBallRequest {
    u_int32_t request_uid;
    u_int8_t from_robot_id;
};
bool operator==(const IncomingBallRequest& a, const IncomingBallRequest& b);

/**
 * @brief request sent by an agent to a specific agent indicating the
 * ball has left this agent's possession and is currently headed for the
 * other agent.
 *
 * an agent will send a BallInTransitRequest to a robot after they have
 * kicked the ball so the other robot knows to begin receiving the ball.
 *
 */
struct BallInTransitRequest {
    u_int32_t request_uid;
    u_int8_t from_robot_id;
};
bool operator==(const BallInTransitRequest& a, const BallInTransitRequest& b);

/**
 * @brief request sent by an agent to determine whether or not it should be going to steal
 * the ball to then shoot the ball
 *
 */
struct ScorerRequest {
    u_int32_t request_uid;
    u_int8_t robot_id;
    double ball_distance;
};
bool operator==(const ScorerRequest& a, const ScorerRequest& b);

/**
 * @brief request sent by a scorer after they shoot to avoid double touch penalties.
 *
 */
struct ResetScorerRequest {
    u_int32_t request_uid;
};
bool operator==(const ResetScorerRequest& a, const ResetScorerRequest& b);

/**
 * @brief request sent by an agent when it wants to join the wall.
 *
 */
struct JoinWallRequest {
    u_int32_t request_uid;
    u_int8_t robot_id;
};
bool operator==(const JoinWallRequest& a, const JoinWallRequest& b);

/**
 * @brief request sent by an agent when it wants to leave the wall.
 *
 */
struct LeaveWallRequest {
    u_int32_t request_uid;
    u_int8_t robot_id;
};
bool operator==(const LeaveWallRequest& a, const LeaveWallRequest& b);

/**
 * @brief a conglomeration of the different request types.
 */
using AgentRequest = std::variant<PassRequest, TestRequest, PositionRequest, IncomingBallRequest,
                                  BallInTransitRequest, ScorerRequest, ResetScorerRequest,
                                  JoinWallRequest, LeaveWallRequest>;

// END REQUEST TYPES //

// BEGIN RESPONSE TYPES //

/**
 * @brief general response given by an agent to any request to let the sender know
 * that this agent has read the message, but will not be sending any additional data.
 *
 * acknowledge can be general purposely used whenever behaviour for a specific
 * request does not need to be specified for a specific role or position.
 *
 */
struct Acknowledge {
    u_int32_t response_uid;
};
bool operator==(const Acknowledge& a, const Acknowledge& b);

/**
 * @brief response from an open agent that will let the sender know that the
 * receiver is open for a pass.
 *
 * the pass response is used to notify the sender that this robot is either
 * open or not open to direct or (in the future) leading passes.
 *
 */
struct PassResponse {
    u_int32_t response_uid;
    bool direct_open;
};
bool operator==(const PassResponse& a, const PassResponse& b);

/**
 * @brief response containing a given agent's position (in string) to a position
 * request from another agent.
 *
 * the position response is used to return the name of the position or role
 * the receiving robot is playing which is currently not used, but could be
 * useful in the future.
 *
 */
struct PositionResponse {
    u_int32_t response_uid;
    std::string position;
};
bool operator==(const PositionResponse& a, const PositionResponse& b);

/**
 * @brief response containing some test message that can be used for testing the
 * sending capabilities between senders and receivers.
 *
 * If things aren't working send the test response with a message of your
 * choosing.
 *
 */
struct TestResponse {
    u_int32_t response_uid;
    std::string message;
};
bool operator==(const TestResponse& a, const TestResponse& b);

/**
 * @brief response to return to another robot asking to be the scorer (only offensive
 * robots will return this response to let another offensive robot know whether or not
 * they should be the scorer)
 *
 */
struct ScorerResponse {
    u_int32_t response_uid;
    u_int8_t robot_id;
    double ball_distance;
};
bool operator==(const ScorerResponse& a, const ScorerResponse& b);

/**
 * @brief response from other robots who are currently apart of the wall so this
 * robot can figure out where they are in the wall of robots.
 *
 */
struct JoinWallResponse {
    u_int32_t response_uid;
    u_int8_t robot_id;
};
bool operator==(const JoinWallResponse& a, const JoinWallResponse& b);

/**
 * @brief response from other robots who are currently apart of the wall.
 *
 */
struct LeaveWallResponse {
    u_int32_t response_uid;
    u_int8_t robot_id;
};
bool operator==(const LeaveWallResponse& a, const LeaveWallResponse& b);

/**
 * @brief conglomeration of the different response types.
 *
 */
using AgentResponseVariant = std::variant<Acknowledge, PassResponse, PositionResponse, TestResponse,
                                          JoinWallResponse, LeaveWallResponse, ScorerResponse>;

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

// END RESPONSE TYPES //

/**
 * @brief Wraps a communication request by giving the intended destination of the communication.
 *
 * positions will create this and send it to their agent action client which will
 * send out the request according to their specifications.
 *
 */
struct PosAgentRequestWrapper {
    AgentRequest request;  // The request to be send
    std::vector<u_int8_t>
        target_agents;  // The target receivers of the message (unnecessary in broadcast)
    bool broadcast;     // Denotes whether or not the message should be sent to all robots.
    bool urgent;        // If urgent, first response is sent through (others are dropped)
};

/**
 * @brief Wraps a communication response to ensure symmetry for agent-to-agent communication.
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
 * @brief Wraps a communication request to ensure symmetry for agent-to-agent communication.
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
    AgentRequest associated_request;           // the request sent for the given response
    std::vector<u_int8_t> to_robot_ids;        // the robot ids the request was sent to
    std::vector<u_int8_t> received_robot_ids;  // the robot ids responses were found from
    bool broadcast;                            // true if the response should go to every robot
    bool urgent;                               // true if only the first response should be handled
    RJ::Time created;                          // the time the request was created
    std::vector<AgentResponseVariant> responses;  // a list of the response from the other agent
};

// TODO (https://app.clickup.com/t/8677c0tqe): Make this templated and less ugly
void generate_uid(PassRequest& request);
void generate_uid(PositionRequest& request);
void generate_uid(TestRequest& request);
void generate_uid(IncomingBallRequest& request);
void generate_uid(BallInTransitRequest& request);
void generate_uid(ScorerRequest& request);
void generate_uid(ResetScorerRequest& request);
void generate_uid(JoinWallRequest& request);
void generate_uid(LeaveWallRequest& request);

void generate_uid(Acknowledge& response);
void generate_uid(PassResponse& response);
void generate_uid(PositionResponse& response);
void generate_uid(TestResponse& response);
void generate_uid(ScorerResponse& response);
void generate_uid(JoinWallResponse& response);
void generate_uid(LeaveWallResponse& response);

}  // namespace strategy::communication

namespace rj_convert {

// BEGIN REQUEST TYPES //

template <>
struct RosConverter<strategy::communication::PassRequest, rj_msgs::msg::PassRequest> {
    static rj_msgs::msg::PassRequest to_ros(const strategy::communication::PassRequest& from) {
        rj_msgs::msg::PassRequest result;
        result.request_uid = from.request_uid;
        result.direct = from.direct;
        result.from_robot_id = from.from_robot_id;
        return result;
    }

    static strategy::communication::PassRequest from_ros(const rj_msgs::msg::PassRequest& from) {
        strategy::communication::PassRequest result{};
        result.request_uid = from.request_uid;
        result.direct = from.direct;
        result.from_robot_id = from.from_robot_id;
        return result;
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::PassRequest, rj_msgs::msg::PassRequest);

template <>
struct RosConverter<strategy::communication::PositionRequest, rj_msgs::msg::PositionRequest> {
    static rj_msgs::msg::PositionRequest to_ros(
        const strategy::communication::PositionRequest& from) {
        rj_msgs::msg::PositionRequest result;
        result.request_uid = from.request_uid;
        return result;
    }

    static strategy::communication::PositionRequest from_ros(
        const rj_msgs::msg::PositionRequest& from) {
        return strategy::communication::PositionRequest{from.request_uid};
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::PositionRequest, rj_msgs::msg::PositionRequest);

template <>
struct RosConverter<strategy::communication::TestRequest, rj_msgs::msg::TestRequest> {
    static rj_msgs::msg::TestRequest to_ros(const strategy::communication::TestRequest& from) {
        rj_msgs::msg::TestRequest result;
        result.request_uid = from.request_uid;
        return result;
    }

    static strategy::communication::TestRequest from_ros(const rj_msgs::msg::TestRequest& from) {
        return strategy::communication::TestRequest{from.request_uid};
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::TestRequest, rj_msgs::msg::TestRequest);

template <>
struct RosConverter<strategy::communication::IncomingBallRequest,
                    rj_msgs::msg::IncomingBallRequest> {
    static rj_msgs::msg::IncomingBallRequest to_ros(
        const strategy::communication::IncomingBallRequest& from) {
        rj_msgs::msg::IncomingBallRequest result;
        result.request_uid = from.request_uid;
        result.from_robot_id = from.from_robot_id;
        return result;
    }

    static strategy::communication::IncomingBallRequest from_ros(
        const rj_msgs::msg::IncomingBallRequest& from) {
        return strategy::communication::IncomingBallRequest{from.request_uid, from.from_robot_id};
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::IncomingBallRequest, rj_msgs::msg::IncomingBallRequest);

template <>
struct RosConverter<strategy::communication::JoinWallRequest, rj_msgs::msg::JoinWallRequest> {
    static rj_msgs::msg::JoinWallRequest to_ros(
        const strategy::communication::JoinWallRequest& from) {
        rj_msgs::msg::JoinWallRequest result;
        result.request_uid = from.request_uid;
        result.robot_id = from.robot_id;
        return result;
    }

    static strategy::communication::JoinWallRequest from_ros(
        const rj_msgs::msg::JoinWallRequest& from) {
        return strategy::communication::JoinWallRequest{from.request_uid, from.robot_id};
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::JoinWallRequest, rj_msgs::msg::JoinWallRequest);

template <>
struct RosConverter<strategy::communication::LeaveWallRequest, rj_msgs::msg::LeaveWallRequest> {
    static rj_msgs::msg::LeaveWallRequest to_ros(
        const strategy::communication::LeaveWallRequest& from) {
        rj_msgs::msg::LeaveWallRequest result;
        result.request_uid = from.request_uid;
        result.robot_id = from.robot_id;
        return result;
    }

    static strategy::communication::LeaveWallRequest from_ros(
        const rj_msgs::msg::LeaveWallRequest& from) {
        return strategy::communication::LeaveWallRequest{from.request_uid, from.robot_id};
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::LeaveWallRequest, rj_msgs::msg::LeaveWallRequest);

template <>
struct RosConverter<strategy::communication::BallInTransitRequest,
                    rj_msgs::msg::BallInTransitRequest> {
    static rj_msgs::msg::BallInTransitRequest to_ros(
        const strategy::communication::BallInTransitRequest& from) {
        rj_msgs::msg::BallInTransitRequest result;
        result.request_uid = from.request_uid;
        result.from_robot_id = from.from_robot_id;
        return result;
    }

    static strategy::communication::BallInTransitRequest from_ros(
        const rj_msgs::msg::BallInTransitRequest& from) {
        return strategy::communication::BallInTransitRequest{from.request_uid, from.from_robot_id};
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::BallInTransitRequest,
                  rj_msgs::msg::BallInTransitRequest);

template <>
struct RosConverter<strategy::communication::ScorerRequest, rj_msgs::msg::ScorerRequest> {
    static rj_msgs::msg::ScorerRequest to_ros(const strategy::communication::ScorerRequest& from) {
        rj_msgs::msg::ScorerRequest result;
        result.request_uid = from.request_uid;
        result.robot_id = from.robot_id;
        result.ball_distance = from.ball_distance;
        return result;
    }

    static strategy::communication::ScorerRequest from_ros(
        const rj_msgs::msg::ScorerRequest& from) {
        return strategy::communication::ScorerRequest{
            from.request_uid,
            from.robot_id,
            from.ball_distance,
        };
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::ScorerRequest, rj_msgs::msg::ScorerRequest);

template <>
struct RosConverter<strategy::communication::ResetScorerRequest, rj_msgs::msg::ResetScorerRequest> {
    static rj_msgs::msg::ResetScorerRequest to_ros(
        const strategy::communication::ResetScorerRequest& from) {
        rj_msgs::msg::ResetScorerRequest result;
        result.request_uid = from.request_uid;
        return result;
    }

    static strategy::communication::ResetScorerRequest from_ros(
        const rj_msgs::msg::ResetScorerRequest& from) {
        return strategy::communication::ResetScorerRequest{
            from.request_uid,
        };
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::ResetScorerRequest, rj_msgs::msg::ResetScorerRequest);

template <>
struct RosConverter<strategy::communication::AgentRequest, rj_msgs::msg::AgentRequest> {
    static rj_msgs::msg::AgentRequest to_ros(const strategy::communication::AgentRequest& from) {
        rj_msgs::msg::AgentRequest result;
        if (const auto* test_request = std::get_if<strategy::communication::TestRequest>(&from)) {
            result.test_request.emplace_back(convert_to_ros(*test_request));
        } else if (const auto* position_request =
                       std::get_if<strategy::communication::PositionRequest>(&from)) {
            result.position_request.emplace_back(convert_to_ros(*position_request));
        } else if (const auto* pass_request =
                       std::get_if<strategy::communication::PassRequest>(&from)) {
            result.pass_request.emplace_back(convert_to_ros(*pass_request));
        } else if (const auto* incoming_ball_request =
                       std::get_if<strategy::communication::IncomingBallRequest>(&from)) {
            result.incoming_ball_request.emplace_back(convert_to_ros(*incoming_ball_request));
        } else if (const auto* ball_in_transit_request =
                       std::get_if<strategy::communication::BallInTransitRequest>(&from)) {
            result.ball_in_transit_request.emplace_back(convert_to_ros(*ball_in_transit_request));
        } else if (const auto* scorer_request =
                       std::get_if<strategy::communication::ScorerRequest>(&from)) {
            result.scorer_request.emplace_back(convert_to_ros(*scorer_request));
        } else if (const auto* reset_scorer_request =
                       std::get_if<strategy::communication::ResetScorerRequest>(&from)) {
            result.reset_scorer_request.emplace_back(convert_to_ros(*reset_scorer_request));
        } else if (const auto* join_wall_request =
                       std::get_if<strategy::communication::JoinWallRequest>(&from)) {
            result.join_wall_request.emplace_back(convert_to_ros(*join_wall_request));
        } else if (const auto* leave_wall_request =
                       std::get_if<strategy::communication::LeaveWallRequest>(&from)) {
            result.leave_wall_request.emplace_back(convert_to_ros(*leave_wall_request));
        } else {
            throw std::runtime_error("Invalid variant of AgentRequest");
        }
        return result;
    }

    static strategy::communication::AgentRequest from_ros(const rj_msgs::msg::AgentRequest& from) {
        strategy::communication::AgentRequest result;
        if (!from.test_request.empty()) {
            result = convert_from_ros(from.test_request.front());
        } else if (!from.position_request.empty()) {
            result = convert_from_ros(from.position_request.front());
        } else if (!from.pass_request.empty()) {
            result = convert_from_ros(from.pass_request.front());
        } else if (!from.incoming_ball_request.empty()) {
            result = convert_from_ros(from.incoming_ball_request.front());
        } else if (!from.ball_in_transit_request.empty()) {
            result = convert_from_ros(from.ball_in_transit_request.front());
        } else if (!from.scorer_request.empty()) {
            result = convert_from_ros(from.scorer_request.front());
        } else if (!from.reset_scorer_request.empty()) {
            result = convert_from_ros(from.reset_scorer_request.front());
        } else if (!from.join_wall_request.empty()) {
            result = convert_from_ros(from.join_wall_request.front());
        } else if (!from.leave_wall_request.empty()) {
            result = convert_from_ros(from.leave_wall_request.front());
        } else {
            throw std::runtime_error("Invalid variant of AgentRequest");
        }
        return result;
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::AgentRequest, rj_msgs::msg::AgentRequest);

// END REQUEST TYPES //

// BEGIN RESPONSE TYPES //

template <>
struct RosConverter<strategy::communication::Acknowledge, rj_msgs::msg::Acknowledge> {
    static rj_msgs::msg::Acknowledge to_ros(const strategy::communication::Acknowledge& from) {
        rj_msgs::msg::Acknowledge result;
        result.response_uid = from.response_uid;
        return result;
    }

    static strategy::communication::Acknowledge from_ros(const rj_msgs::msg::Acknowledge& from) {
        return strategy::communication::Acknowledge{from.response_uid};
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::Acknowledge, rj_msgs::msg::Acknowledge);

template <>
struct RosConverter<strategy::communication::PassResponse, rj_msgs::msg::PassResponse> {
    static rj_msgs::msg::PassResponse to_ros(const strategy::communication::PassResponse& from) {
        rj_msgs::msg::PassResponse result;
        result.response_uid = from.response_uid;
        result.direct_open = from.direct_open;
        return result;
    }

    static strategy::communication::PassResponse from_ros(const rj_msgs::msg::PassResponse& from) {
        strategy::communication::PassResponse result{};
        result.response_uid = from.response_uid;
        result.direct_open = from.direct_open;
        return result;
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::PassResponse, rj_msgs::msg::PassResponse);

template <>
struct RosConverter<strategy::communication::PositionResponse, rj_msgs::msg::PositionResponse> {
    static rj_msgs::msg::PositionResponse to_ros(
        const strategy::communication::PositionResponse& from) {
        rj_msgs::msg::PositionResponse result;
        result.position = from.position;
        result.response_uid = from.response_uid;
        return result;
    }

    static strategy::communication::PositionResponse from_ros(
        const rj_msgs::msg::PositionResponse& from) {
        strategy::communication::PositionResponse result;
        result.position = from.position;
        result.response_uid = from.response_uid;
        return result;
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::PositionResponse, rj_msgs::msg::PositionResponse);

template <>
struct RosConverter<strategy::communication::TestResponse, rj_msgs::msg::TestResponse> {
    static rj_msgs::msg::TestResponse to_ros(const strategy::communication::TestResponse& from) {
        rj_msgs::msg::TestResponse result;
        result.message = from.message;
        result.response_uid = from.response_uid;
        return result;
    }

    static strategy::communication::TestResponse from_ros(const rj_msgs::msg::TestResponse& from) {
        strategy::communication::TestResponse result;
        result.message = from.message;
        result.response_uid = from.response_uid;
        return result;
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::TestResponse, rj_msgs::msg::TestResponse);

template <>
struct RosConverter<strategy::communication::ScorerResponse, rj_msgs::msg::ScorerResponse> {
    static rj_msgs::msg::ScorerResponse to_ros(
        const strategy::communication::ScorerResponse& from) {
        rj_msgs::msg::ScorerResponse result;
        result.response_uid = from.response_uid;
        result.robot_id = from.robot_id;
        result.ball_distance = from.ball_distance;
        return result;
    }

    static strategy::communication::ScorerResponse from_ros(
        const rj_msgs::msg::ScorerResponse& from) {
        strategy::communication::ScorerResponse result{from.response_uid, from.robot_id,
                                                       from.ball_distance};
        return result;
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::ScorerResponse, rj_msgs::msg::ScorerResponse);

template <>
struct RosConverter<strategy::communication::JoinWallResponse, rj_msgs::msg::JoinWallResponse> {
    static rj_msgs::msg::JoinWallResponse to_ros(
        const strategy::communication::JoinWallResponse& from) {
        rj_msgs::msg::JoinWallResponse result;
        result.response_uid = from.response_uid;
        result.robot_id = from.robot_id;
        return result;
    }

    static strategy::communication::JoinWallResponse from_ros(
        const rj_msgs::msg::JoinWallResponse& from) {
        return strategy::communication::JoinWallResponse{from.response_uid, from.robot_id};
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::JoinWallResponse, rj_msgs::msg::JoinWallResponse);

template <>
struct RosConverter<strategy::communication::LeaveWallResponse, rj_msgs::msg::LeaveWallResponse> {
    static rj_msgs::msg::LeaveWallResponse to_ros(
        const strategy::communication::LeaveWallResponse& from) {
        rj_msgs::msg::LeaveWallResponse result;
        result.response_uid = from.response_uid;
        result.robot_id = from.robot_id;
        return result;
    }

    static strategy::communication::LeaveWallResponse from_ros(
        const rj_msgs::msg::LeaveWallResponse& from) {
        return strategy::communication::LeaveWallResponse{from.response_uid, from.robot_id};
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::LeaveWallResponse, rj_msgs::msg::LeaveWallResponse);

template <>
struct RosConverter<strategy::communication::AgentResponse, rj_msgs::msg::AgentResponse> {
    static rj_msgs::msg::AgentResponse to_ros(const strategy::communication::AgentResponse& from) {
        rj_msgs::msg::AgentResponse result;
        result.associated_request = convert_to_ros(from.associated_request);
        if (const auto* acknowledge_response =
                std::get_if<strategy::communication::Acknowledge>(&(from.response))) {
            result.response.acknowledge_response.emplace_back(
                convert_to_ros(*acknowledge_response));
        } else if (const auto* test_response =
                       std::get_if<strategy::communication::TestResponse>(&(from.response))) {
            result.response.test_response.emplace_back(convert_to_ros(*test_response));
        } else if (const auto* position_response =
                       std::get_if<strategy::communication::PositionResponse>(&(from.response))) {
            result.response.position_response.emplace_back(convert_to_ros(*position_response));
        } else if (const auto* pass_response =
                       std::get_if<strategy::communication::PassResponse>(&(from.response))) {
            result.response.pass_response.emplace_back(convert_to_ros(*pass_response));
        } else if (const auto* scorer_response =
                       std::get_if<strategy::communication::ScorerResponse>(&(from.response))) {
            result.response.scorer_response.emplace_back(convert_to_ros(*scorer_response));
        } else if (const auto* join_wall_response =
                       std::get_if<strategy::communication::JoinWallResponse>(&(from.response))) {
            result.response.join_wall_response.emplace_back(convert_to_ros(*join_wall_response));
        } else if (const auto* leave_wall_response =
                       std::get_if<strategy::communication::LeaveWallResponse>(&(from.response))) {
            result.response.leave_wall_response.emplace_back(convert_to_ros(*leave_wall_response));
        } else {
            throw std::runtime_error("Invalid variant of AgentResponse");
        }
        return result;
    }

    static strategy::communication::AgentResponse from_ros(
        const rj_msgs::msg::AgentResponse& from) {
        strategy::communication::AgentResponse result;
        result.associated_request = convert_from_ros(from.associated_request);
        if (!from.response.acknowledge_response.empty()) {
            result.response = convert_from_ros(from.response.acknowledge_response.front());
        } else if (!from.response.test_response.empty()) {
            result.response = convert_from_ros(from.response.test_response.front());
        } else if (!from.response.position_response.empty()) {
            result.response = convert_from_ros(from.response.position_response.front());
        } else if (!from.response.pass_response.empty()) {
            result.response = convert_from_ros(from.response.pass_response.front());
        } else if (!from.response.scorer_response.empty()) {
            result.response = convert_from_ros(from.response.scorer_response.front());
        } else if (!from.response.join_wall_response.empty()) {
            result.response = convert_from_ros(from.response.join_wall_response.front());
        } else if (!from.response.leave_wall_response.empty()) {
            result.response = convert_from_ros(from.response.leave_wall_response.front());
        } else {
            throw std::runtime_error("Invalid variant of AgentResponse");
        }
        return result;
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::AgentResponse, rj_msgs::msg::AgentResponse);

// END RESPONSE TYPES //

}  // namespace rj_convert