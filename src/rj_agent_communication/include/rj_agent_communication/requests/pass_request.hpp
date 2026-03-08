#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_agent_msgs/msg/pass_request.hpp"

namespace communication {

struct PassRequest {
    uint32_t request_uid;
    bool direct;
    uint8_t from_robot_id;
};

//NOLINTNEXTLINE(readability-identifier-length)
bool operator==(const PassRequest& a, const PassRequest& b);
void generate_uid(PassRequest& request);

}  // namespace communication

namespace rj_convert {

template <>
struct RosConverter<communication::PassRequest, rj_agent_msgs::msg::PassRequest> {
    static rj_agent_msgs::msg::PassRequest to_ros(const communication::PassRequest& from) {
        rj_agent_msgs::msg::PassRequest result;
        result.request_uid = from.request_uid;
        result.direct = from.direct;
        result.from_robot_id = from.from_robot_id;
        return result;
    }

    static communication::PassRequest from_ros(const rj_agent_msgs::msg::PassRequest& from) {
        return communication::PassRequest{
            from.request_uid,
            from.direct,
            from.from_robot_id,
        };
    }
};

ASSOCIATE_CPP_ROS(communication::PassRequest, rj_agent_msgs::msg::PassRequest);

}  // namespace rj_convert