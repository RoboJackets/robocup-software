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

namespace rclcpp {

template <>
struct TypeAdapter<strategy::communication::JoinWallRequest, rj_msgs::msg::JoinWallRequest> {
    using is_specialized = std::true_type;
    using custom_type = strategy::communication::JoinWallRequest;
    using ros_message_type = rj_msgs::msg::JoinWallRequest;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination.request_uid = source.request_uid;
        destination.robot_id = source.robot_id;
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = strategy::communication::JoinWallRequest{
            source.request_uid, source.robot_id};
    }
};


}  // namespace rclcpp