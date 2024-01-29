#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>

#include "rj_msgs/msg/join_wall_request.hpp"

namespace strategy::communication {

struct JoinWallRequest {
    uint32_t request_uid;
    uint8_t robot_id;
};

bool operator==(const JoinWallRequest& a, const JoinWallRequest& b);
void generate_uid(JoinWallRequest& request);

}  // namespace strategy::communication

namespace rj_convert {

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
        return strategy::communication::JoinWallRequest{
            from.request_uid,
            from.robot_id,
        };
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::JoinWallRequest, rj_msgs::msg::JoinWallRequest);

}  // namespace rj_convert