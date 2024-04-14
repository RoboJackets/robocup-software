#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/leave_wall_request.hpp"

namespace strategy::communication {

struct LeaveWallRequest {
    uint32_t request_uid;
    uint8_t robot_id;
};

bool operator==(const LeaveWallRequest& a, const LeaveWallRequest& b);
void generate_uid(LeaveWallRequest& request);

}  // namespace strategy::communication

namespace rj_convert {

template <>
struct RosConverter<strategy::communication::LeaveWallRequest, rj_msgs::msg::LeaveWallRequest> {
    static rj_msgs::msg::LeaveWallRequest to_ros(
        const strategy::communication::LeaveWallRequest& from) {
        rj_msgs::msg::LeaveWallRequest result;
        result.request_uid = from.request_uid;
        result.robot_id = from.robot_id;
        return result;
    }

    static strategy::communication::LeaveWallRequest from_ros(
        const rj_msgs::msg::LeaveWallRequest& from) {
        return strategy::communication::LeaveWallRequest{
            from.request_uid,
            from.robot_id,
        };
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::LeaveWallRequest, rj_msgs::msg::LeaveWallRequest);

}  // namespace rj_convert