#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>

#include "rj_msgs/msg/pass_response.hpp"

namespace strategy::communication {

struct PassResponse {
    uint32_t response_uid;
    bool direct_open;
};

bool operator==(const PassResponse& a, const PassResponse& b);
void generate_uid(PassResponse& response);

}  // namespace strategy::communication

namespace rj_convert {

template <>
struct RosConverter<strategy::communication::PassResponse, rj_msgs::msg::PassResponse> {
    static rj_msgs::msg::PassResponse to_ros(const strategy::communication::PassResponse& from) {
        rj_msgs::msg::PassResponse result;
        result.response_uid = from.response_uid;
        result.direct_open = from.direct_open;
        return result;
    }

    static strategy::communication::PassResponse from_ros(const rj_msgs::msg::PassResponse& from) {
        return strategy::communication::PassResponse{
            from.response_uid,
            from.direct_open,
        };
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::PassResponse, rj_msgs::msg::PassResponse);

}  // namespace rj_convert