#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_agent_msgs/msg/pass_response.hpp"

namespace communication {

struct PassResponse {
    uint32_t response_uid;
    bool direct_open;
};

//NOLINTNEXTLINE(readability-identifier-length)
bool operator==(const PassResponse& a, const PassResponse& b);
void generate_uid(PassResponse& response);

}  // namespace communication

namespace rj_convert {

template <>
struct RosConverter<communication::PassResponse, rj_agent_msgs::msg::PassResponse> {
    static rj_agent_msgs::msg::PassResponse to_ros(const communication::PassResponse& from) {
        rj_agent_msgs::msg::PassResponse result;
        result.response_uid = from.response_uid;
        result.direct_open = from.direct_open;
        return result;
    }

    static communication::PassResponse from_ros(const rj_agent_msgs::msg::PassResponse& from) {
        return communication::PassResponse{
            from.response_uid,
            from.direct_open,
        };
    }
};

ASSOCIATE_CPP_ROS(communication::PassResponse, rj_agent_msgs::msg::PassResponse);

}  // namespace rj_convert