#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_agent_msgs/msg/position_response.hpp"

namespace communication {

struct PositionResponse {
    uint32_t response_uid;
    std::string position;
};

//NOLINTNEXTLINE(readability-identifier-length)
bool operator==(const PositionResponse& a, const PositionResponse& b);
void generate_uid(PositionResponse& response);

}  // namespace communication

namespace rj_convert {

template <>
struct RosConverter<communication::PositionResponse, rj_agent_msgs::msg::PositionResponse> {
    static rj_agent_msgs::msg::PositionResponse to_ros(
        const communication::PositionResponse& from) {
        rj_agent_msgs::msg::PositionResponse result;
        result.response_uid = from.response_uid;
        result.position = from.position;
        return result;
    }

    static communication::PositionResponse from_ros(
        const rj_agent_msgs::msg::PositionResponse& from) {
        return communication::PositionResponse{
            from.response_uid,
            from.position,
        };
    }
};

ASSOCIATE_CPP_ROS(communication::PositionResponse, rj_agent_msgs::msg::PositionResponse);

}  // namespace rj_convert