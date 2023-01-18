#pragma once

#include <string>
#include <variant>
#include <vector>

#include <rj_convert/rj_convert.hpp>

#include "rj_msgs/msg/agent_request.hpp"
#include "rj_msgs/msg/agent_response.hpp"
#include "rj_msgs/msg/agent_response_variant.hpp"
#include "rj_msgs/msg/pass_request.hpp"
#include "rj_msgs/msg/position_request.hpp"
#include "rj_msgs/msg/test_request.hpp"
#include "rj_msgs/msg/acknowledge.hpp"
#include "rj_msgs/msg/pass_response.hpp"
#include "rj_msgs/msg/position_response.hpp"
#include "rj_msgs/msg/test_response.hpp"

namespace strategy {

namespace communication {
    /**
     * @brief Wraps a communication request by giving the intended destination of the communication.
     * 
     */
    struct PosAgentRequestWrapper {
        AgentRequest request; // The request to be send
        std::optional<std::vector<uint8_t>> target_agents; // The target receivers of the message (unnecessary in broadcast)
        bool broadcast; // Denotes whether or not the message should be sent to all robots.
        bool urgent; // If urgent, first response is sent through (others are dropped)
    };

    /**
     * @brief Wraps a communication response to ensure symmetry for agent-to-agent communication.
     * 
     */
    struct PosAgentResponseWrapper {
        AgentResponse response;
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
        std::vector<AgentResponseVariant> responses;
    };

    // BEGIN REQUEST TYPES //

    struct PassRequest {};
    bool operator==(const PassRequest& a, const PassRequest& b);

    struct PositionRequest {};
    bool operator==(const PositionRequest& a, const PositionRequest& b);

    struct TestRequest {};
    bool operator==(const TestRequest& a, const TestRequest& b);

    using AgentRequest = std::variant<PassRequest, TestRequest, PositionRequest>;

    // END REQUEST TYPES //

    // BEGIN RESPONSE TYPES //

    struct Acknowledge {
        bool acknowledged;
    };
    bool operator==(const AcknowledgeResponse& a, const AcknowledgeResponse& b);

    struct PassResponse {};
    bool operator==(const PassResponse& a, const PassResponse& b);

    struct PositionResponse {
        std::string position;
    };
    bool operator==(const PositionResponse& a, const PositionResponse& b);

    struct TestResponse {
        std::string message;
    };
    bool operator==(const TestResponse& a, const TestResponse& b);

    using AgentResponseVariant = std::variant<Acknowledge, PassResponse, PositionResponse, TestResponse>;

    struct AgentResponse {
        AgentRequest associated_request;
        AgentResponseVariant response;
    };
    bool operator==(const AgentResponse& a, const AgentResponse& b);

    // END RESPONSE TYPES //

} // namespace communication

} // namespace strategy

namespace rj_convert {

// BEGIN REQUEST TYPES //

template <>
struct RosConvert<strategy::communication::PassRequest, rj_msgs::msg::PassRequest> {
    static rj_msgs::msg::PassRequest to_ros(
        [[maybe_unused]] const strategy::communication::PassRequest& from) {
        return rj_msgs::msg::PassRequest{};
    }

    static strategy::communication::PassRequest from_ros(
        [[maybe_unused]] const rj_msgs::msg::PassRequest& from) {
        return strategy::communication::PassRequest{};
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::PassRequest, rj_msgs::msg::PassRequest);

template <>
struct RosConvert<strategy::communication::PositionRequest, rj_msgs::msg::PositionRequest> {
    static rj_msgs::msg::PositionRequest to_ros(
        [[maybe_unused]] const strategy::communication::PositionRequest& from) {
        return rj_msgs::msg::PositionRequest{};
    }

    static strategy::communication::PositionRequest from_ros(
        [[maybe_unused]] const rj_msgs::msg::PositionRequest& from) {
        return strategy::communication::PositionRequest{};
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::PositionRequest, rj_msgs::msg::PositionRequest);

template <>
struct RosConvert<strategy::communication::TestRequest, rj_msgs::msg::TestRequest> {
    static rj_msgs::msg::TestRequest to_ros(
        [[maybe_unused]] const strategy::communication::TestRequest& from) {
        return rj_msgs::msg::TestRequest{};
    }

    static strategy::communication::TestRequest from_ros(
        [[maybe_unused]] const rj_msgs::msg::TestRequest& from) {
        return strategy::communication::TestRequest{};
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::TestRequest, rj_msgs::msg::TestRequest);

template <>
struct RosConvert<strategy::communication::AgentRequest, rj_msgs::msg::AgentRequest> {
    static rj_msgs::msg::AgentRequest to_ros(
        const strategy::communication::AgentRequest& from) {
        rj_msgs::msg::AgentRequest result;
        if (const auto* test_request = std::get_if<strategy::communication::TestRequest>(&from)) {
            result.test_request.emplace_back(convert_to_ros(*test_request));
        } else if (const auto* position_request = std::get_if<strategy::communication::PositionRequest>(&from)) {
            result.position_request.emplace_back(convert_to_ros(*position_request));
        } else if (const auto* pass_request = std::get_if<strategy::communication::PassRequest>(&from)) {
            result.pass_request.emplace_back(convert_to_ros(*pass_request));
        } else {
            throw std::runtime_error("Invalid variant of AgentRequest");
        }
        return result;
    }

    static strategy::communication::AgentRequest from_ros(
        const rj_msgs::msg::AgentRequest& from) {
        strategy::communication::AgentRequest result;
        if (!from.test_request.empty()) {
            result = convert_from_ros(from.test_request.front());
        } else if (!from.position_request.empty()) {
            result = convert_from_ros(from.position_request.front());
        } else if (!from.pass_request.empty()) {
            result = convert_from_ros(from.pass_request.front());
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
struct RosConvert<strategy::communication::Acknowledge, rj_msgs::msg::Acknowledge> {
    static rj_msgs::msg::Acknowledge to_ros(
        const strategy::communication::Acknowledge& from) {
        rj_msgs::msg::Acknowledge result;
        result.acknowledged = from.acknowledged;
        return result;
    }

    static strategy::communication::Acknowledge from_ros(
        const rj_msgs::msg::Acknowledge& from) {
        strategy::communication::Acknowledge result;
        result.acknowledged = from.acknowledged;
        return result
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::Acknowledge, rj_msgs::msg::Acknowledge);

template <>
struct RosConvert<strategy::communication::PassResponse, rj_msgs::msg::PassResponse> {
    static rj_msgs::msg::PassResponse to_ros(
        const strategy::communication::PassResponse& from) {
        rj_msgs::msg::PassResponse result;
        return result;
    }

    static strategy::communication::PassResponse from_ros(
        const rj_msgs::msg::PassResponse& from) {
        strategy::communication::PassResponse result;
        return result;
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::PassResponse, rj_msgs::msg::PassResponse);

template <>
struct RosConvert<strategy::communication::PositionResponse, rj_msgs::msg::PositionResponse> {
    static rj_msgs::msg::PositionResponse to_ros(
        const strategy::communication::PositionResponse& from) {
        rj_msgs::msg::PositionResponse result;
        result.position = from.position;
        return result;
    }

    static strategy::communication::PositionResponse from_ros(
        const rj_msgs::msg::PositionResponse& from) {
        strategy::communication::PositionResponse result;
        result.position = from.position;
        return result;
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::PositionResponse, rj_msgs::msg::PositionResponse);

template <>
struct RosConvert<strategy::communication::TestResponse, rj_msgs::msg::TestResponse> {
    static rj_msgs::msg::TestResponse to_ros(
        const strategy::communication::TestResponse& from) {
        rj_msgs::msg::TestResponse result;
        result.message = from.message;
        return result;
    }

    static strategy::communication::TestResponse from_ros(
        const rj_msgs::msg::TestResponse& from) {
        strategy::communication::TestResponse result;
        result.message = from.message;
        return result;
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::TestResponse, rj_msgs::msg::TestResponse);

template <>
struct RosConvert<strategy::communication::AgentResponse, rj_msgs::msg::AgentResponse> {
    static rj_msgs::msg::AgentResponse to_ros(
        const strategy::communication::AgentResponse& from) {
        rj_msgs::msg::AgentResponse result;
        result.associated_request = convert_to_ros(from.associated_request);
        if (const auto* acknowledge_response = std::get_if<strategy::communication::Acknowledge>(&(from.response))) {
            result.response.acknowledge_response.emplace_back(convert_to_ros(*acknowledge_response));
        } else if (const auto* test_response = std::get_if<strategy::communication::TestResponse>(&(from.response))) {
            result.response.test_response.emplace_back(convert_to_ros(*test_response));
        } else if (const auto* position_response = std::get_if<strategy::communication::PositionResponse>(&(from.response))) {
            result.response.position_response.emplace_back(convert_to_ros(*position_response));
        } else if (const auto* pass_response = std::get_if<strategy::communication::PassResponse>(&(from.response))) {
            result.response.pass_response.emplace_back(convert_to_ros(*pass_response));
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
        } else {
            throw std::runtime_error("Invalid variant of AgentResponse");
        }
        return result;
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::AgentResponse, rj_msgs::msg::AgentResponse);

// END RESPONSE TYPES //

} // namespace rj_convert