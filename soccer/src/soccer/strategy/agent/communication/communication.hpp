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
#include "rj_msgs/msg/incoming_pass_request.hpp"
#include "rj_msgs/msg/pass_request.hpp"
#include "rj_msgs/msg/pass_response.hpp"
#include "rj_msgs/msg/position_request.hpp"
#include "rj_msgs/msg/position_response.hpp"
#include "rj_msgs/msg/test_request.hpp"
#include "rj_msgs/msg/test_response.hpp"
#include "rj_msgs/msg/join_wall_request.hpp"
#include "rj_msgs/msg/join_wall_response.hpp"
#include "rj_msgs/msg/leave_wall_request.hpp"
#include "rj_msgs/msg/leave_wall_response.hpp"

namespace strategy::communication {

// BEGIN REQUEST TYPES //

struct PassRequest {
    u_int32_t request_uid;
    bool direct;
    u_int8_t from_robot_id;
};
bool operator==(const PassRequest& a, const PassRequest& b);

struct PositionRequest {
    u_int32_t request_uid;
};
bool operator==(const PositionRequest& a, const PositionRequest& b);

struct TestRequest {
    u_int32_t request_uid;
};
bool operator==(const TestRequest& a, const TestRequest& b);

struct IncomingPassRequest {
    u_int32_t request_uid;
    u_int8_t from_robot_id;
};
bool operator==(const IncomingPassRequest& a, const IncomingPassRequest& b);

struct BallInTransitRequest {
    u_int32_t request_uid;
    u_int8_t from_robot_id;
};
bool operator==(const BallInTransitRequest& a, const BallInTransitRequest& b);

struct JoinWallRequest {
    u_int32_t request_uid;
    u_int8_t robot_id;
};
bool operator==(const JoinWallRequest& a, const JoinWallRequest& b);

struct LeaveWallRequest {
    u_int32_t request_uid;
    u_int8_t robot_id;
};
bool operator==(const LeaveWallRequest& a, const LeaveWallRequest& b);

using AgentRequest = std::variant<PassRequest, TestRequest, PositionRequest, IncomingPassRequest,
                                  BallInTransitRequest, JoinWallRequest, LeaveWallRequest>;

// END REQUEST TYPES //

// BEGIN RESPONSE TYPES //

struct Acknowledge {
    u_int32_t response_uid;
};
bool operator==(const Acknowledge& a, const Acknowledge& b);

struct PassResponse {
    u_int32_t response_uid;
    bool direct_open;
};
bool operator==(const PassResponse& a, const PassResponse& b);

struct PositionResponse {
    u_int32_t response_uid;
    std::string position;
};
bool operator==(const PositionResponse& a, const PositionResponse& b);

struct TestResponse {
    u_int32_t response_uid;
    std::string message;
};
bool operator==(const TestResponse& a, const TestResponse& b);

struct JoinWallResponse {
    u_int32_t response_uid;
    u_int8_t robot_id;
};
bool operator==(const JoinWallResponse& a, const JoinWallResponse& b);

struct LeaveWallResponse {
    u_int32_t response_uid;
    u_int8_t robot_id;
};
bool operator==(const LeaveWallResponse& a, const LeaveWallResponse& b);

using AgentResponseVariant =
    std::variant<Acknowledge, PassResponse, PositionResponse, TestResponse, JoinWallResponse, LeaveWallResponse>;

struct AgentResponse {
    AgentRequest associated_request;
    AgentResponseVariant response;
};
bool operator==(const AgentResponse& a, const AgentResponse& b);

// END RESPONSE TYPES //

/**
 * @brief Wraps a communication request by giving the intended destination of the communication.
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
 */
struct PosAgentResponseWrapper {
    AgentResponseVariant response;
};

/**
 * @brief Wraps a communication request to ensure symmetry for agent-to-agent communication.
 *
 */
struct AgentPosRequestWrapper {
    AgentRequest request;
};

/**
 * @brief Wraps a communication response by giving the robot the communication is from.
 *
 */
struct AgentPosResponseWrapper {
    AgentRequest associated_request;
    std::vector<u_int8_t> from_robot_ids;
    std::vector<u_int8_t> received_robot_ids;
    bool broadcast;
    bool urgent;
    RJ::Time created;
    std::vector<AgentResponseVariant> responses;
};

// TODO (https://app.clickup.com/t/8677c0tqe): Make this templated and less ugly
void generate_uid(PassRequest& request);
void generate_uid(PositionRequest& request);
void generate_uid(TestRequest& request);
void generate_uid(IncomingPassRequest& request);
void generate_uid(BallInTransitRequest& request);
void generate_uid(JoinWallRequest& request);
void generate_uid(LeaveWallRequest& request);

void generate_uid(Acknowledge& response);
void generate_uid(PassResponse& response);
void generate_uid(PositionResponse& response);
void generate_uid(TestResponse& response);
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
struct RosConverter<strategy::communication::IncomingPassRequest,
                    rj_msgs::msg::IncomingPassRequest> {
    static rj_msgs::msg::IncomingPassRequest to_ros(
        const strategy::communication::IncomingPassRequest& from) {
        rj_msgs::msg::IncomingPassRequest result;
        result.request_uid = from.request_uid;
        result.from_robot_id = from.from_robot_id;
        return result;
    }

    static strategy::communication::IncomingPassRequest from_ros(
        const rj_msgs::msg::IncomingPassRequest& from) {
        return strategy::communication::IncomingPassRequest{from.request_uid, from.from_robot_id};
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::IncomingPassRequest, rj_msgs::msg::IncomingPassRequest);

template <>
struct RosConverter<strategy::communication::JoinWallRequest,
                    rj_msgs::msg::JoinWallRequest> {
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
struct RosConverter<strategy::communication::LeaveWallRequest,
                    rj_msgs::msg::LeaveWallRequest> {
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
        } else if (const auto* incoming_ass_request =
                       std::get_if<strategy::communication::IncomingPassRequest>(&from)) {
            result.incoming_pass_request.emplace_back(convert_to_ros(*incoming_ass_request));
        } else if (const auto* ball_in_transit_request =
                       std::get_if<strategy::communication::BallInTransitRequest>(&from)) {
            result.ball_in_transit_request.emplace_back(convert_to_ros(*ball_in_transit_request));
        } else if (const auto* join_wall_request = std::get_if<strategy::communication::JoinWallRequest>(&from)) {
            result.join_wall_request.emplace_back(convert_to_ros(*join_wall_request));
        } else if (const auto* leave_wall_request = std::get_if<strategy::communication::LeaveWallRequest>(&from)) {
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
        } else if (!from.incoming_pass_request.empty()) {
            result = convert_from_ros(from.incoming_pass_request.front());
        } else if (!from.ball_in_transit_request.empty()) {
            result = convert_from_ros(from.ball_in_transit_request.front());
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
struct RosConverter<strategy::communication::JoinWallResponse,
                    rj_msgs::msg::JoinWallResponse> {
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
struct RosConverter<strategy::communication::LeaveWallResponse,
                    rj_msgs::msg::LeaveWallResponse> {
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
        } else if (const auto* join_wall_response = std::get_if<strategy::communication::JoinWallResponse>(&(from.response))) {
            result.response.join_wall_response.emplace_back(convert_to_ros(*join_wall_response));
        } else if (const auto* leave_wall_response = std::get_if<strategy::communication::LeaveWallResponse>(&(from.response))) {
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