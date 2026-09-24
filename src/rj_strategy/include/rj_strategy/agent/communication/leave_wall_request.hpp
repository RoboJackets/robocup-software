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

namespace rclcpp {

template <>
struct TypeAdapter<strategy::communication::LeaveWallRequest, rj_msgs::msg::LeaveWallRequest> {
    using is_specialized = std::true_type;
    using custom_type = strategy::communication::LeaveWallRequest;
    using ros_message_type = rj_msgs::msg::LeaveWallRequest;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination.request_uid = source.request_uid;
        destination.robot_id = source.robot_id;
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = strategy::communication::LeaveWallRequest{
            source.request_uid, source.robot_id};
    }
};


}  // namespace rclcpp