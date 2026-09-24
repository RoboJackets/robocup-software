#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/leave_wall_response.hpp"

namespace strategy::communication {

struct LeaveWallResponse {
    uint32_t response_uid;
    uint8_t robot_id;
};

bool operator==(const LeaveWallResponse& a, const LeaveWallResponse& b);
void generate_uid(LeaveWallResponse& response);

}  // namespace strategy::communication

namespace rclcpp {

template <>
struct TypeAdapter<strategy::communication::LeaveWallResponse, rj_msgs::msg::LeaveWallResponse> {
    using is_specialized = std::true_type;
    using custom_type = strategy::communication::LeaveWallResponse;
    using ros_message_type = rj_msgs::msg::LeaveWallResponse;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination.response_uid = source.response_uid;
        destination.robot_id = source.robot_id;
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = strategy::communication::LeaveWallResponse{
            source.response_uid, source.robot_id};
    }
};


}  // namespace rclcpp