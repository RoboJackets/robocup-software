#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_agent_msgs/msg/position_request.hpp"

namespace communication {

struct PositionRequest {
    uint32_t request_uid;
};

//NOLINTNEXTLINE(readability-identifier-length)
bool operator==(const PositionRequest& a, const PositionRequest& b);
void generate_uid(PositionRequest& request);

}  // namespace communication

namespace rj_convert {

template <>
struct RosConverter<communication::PositionRequest, rj_agent_msgs::msg::PositionRequest> {
    static rj_agent_msgs::msg::PositionRequest to_ros(
        const communication::PositionRequest& from) {
        rj_agent_msgs::msg::PositionRequest result;
        result.request_uid = from.request_uid;
        return result;
    }

    static communication::PositionRequest from_ros(
        const rj_agent_msgs::msg::PositionRequest& from) {
        return communication::PositionRequest{
            from.request_uid,
        };
    }
};

ASSOCIATE_CPP_ROS(communication::PositionRequest, rj_agent_msgs::msg::PositionRequest);

}  // namespace rj_convert