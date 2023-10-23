#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>

#include "rj_msgs/msg/position_response.hpp"

namespace strategy::communication {

struct PositionResponse {
    uint32_t response_uid;
    std::string position;
};

bool operator==(const PositionResponse& a, const PositionResponse& b);
void generate_uid(PositionResponse& response);

}  // namespace strategy::communication

namespace rj_convert {

template <>
struct RosConverter<strategy::communication::PositionResponse, rj_msgs::msg::PositionResponse> {
    static rj_msgs::msg::PositionResponse to_ros(
        const strategy::communication::PositionResponse& from) {
        rj_msgs::msg::PositionResponse result;
        result.response_uid = from.response_uid;
        result.position = from.position;
        return result;
    }

    static strategy::communication::PositionResponse from_ros(
        const rj_msgs::msg::PositionResponse& from) {
        return strategy::communication::PositionResponse{
            from.response_uid,
            from.position,
        };
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::PositionResponse, rj_msgs::msg::PositionResponse);

}  // namespace rj_convert