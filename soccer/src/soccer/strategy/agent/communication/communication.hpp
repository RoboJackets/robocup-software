#pragma once

#include "rj_msgs/msg/agent_request.hpp"
#include "rj_msgs/msg/agent_response.hpp"

namespace strategy {

namespace communication {
    /**
     * @brief CommunicationType stands for the type of communication received or transmitted during
     * agent to agent communication.
     * 
     */
    enum CommunicationType {
        acknowledge, // Default response for unhandeled communication requests
        test, // Request and Response type for test messages (a returned string)
        position, // Request and Response type for robots getting other robots positions (mostly used for testing)
        pass, // Request and Response type for robots attempting to make a pass to other robots
    };

    /**
     * @brief Wraps a communication request by giving the intended destination of the communication.
     * 
     */
    struct PosAgentRequestWrapper {
        rj_msgs::msg::AgentRequest request; // The request to be send
        std::optional<std::vector<uint8_t>> target_agents; // The target receivers of the message (unnecessary in broadcast)
        bool broadcast; // Denotes whether or not the message should be sent to all robots.
    };

    /**
     * @brief Wraps a communication response to ensure symmetry for agent-to-agent communication.
     * 
     */
    struct PosAgentResponseWrapper {
        rj_msgs::msg::AgentResponse response;
    };

    /**
     * @brief Wraps a communication request to ensure symmetry for agent-to-agent communication.
     * 
     */
    struct AgentPosRequestWrapper {
        rj_msgs::msg::AgentRequest request;
    };

    /**
     * @brief Wraps a communication response by giving the robot the communication is from.
     * 
     */
    struct AgentPosResponseWrapper {
        u_int8_t from_robot_id;
        rj_msgs::msg::AgentResponse response;
    };
}

}