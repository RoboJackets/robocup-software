#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_agent_msgs/msg/leave_wall_response.hpp"

namespace communication {

struct LeaveWallResponse {
    uint32_t response_uid;
    uint8_t robot_id;
};

//NOLINTNEXTLINE(readability-identifier-length)
bool operator==(const LeaveWallResponse& a, const LeaveWallResponse& b);
void generate_uid(LeaveWallResponse& response);

}  // namespace communication

namespace rj_convert {

template <>
struct RosConverter<communication::LeaveWallResponse, rj_agent_msgs::msg::LeaveWallResponse> {
    static rj_agent_msgs::msg::LeaveWallResponse to_ros(
        const communication::LeaveWallResponse& from) {
        rj_agent_msgs::msg::LeaveWallResponse result;
        result.response_uid = from.response_uid;
        result.robot_id = from.robot_id;
        return result;
    }

    static communication::LeaveWallResponse from_ros(
        const rj_agent_msgs::msg::LeaveWallResponse& from) {
        return communication::LeaveWallResponse{
            from.response_uid,
            from.robot_id,
        };
    }
};

ASSOCIATE_CPP_ROS(communication::LeaveWallResponse, rj_agent_msgs::msg::LeaveWallResponse);

}  // namespace rj_convert