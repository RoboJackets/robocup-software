#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/join_wall_response.hpp"

namespace strategy::communication {

struct JoinWallResponse {
    uint32_t response_uid;
    uint8_t robot_id;
};

bool operator==(const JoinWallResponse& a, const JoinWallResponse& b);
void generate_uid(JoinWallResponse& response);

}  // namespace strategy::communication

namespace rclcpp {

template <>
struct TypeAdapter<strategy::communication::JoinWallResponse, rj_msgs::msg::JoinWallResponse> {
    using is_specialized = std::true_type;
    using custom_type = strategy::communication::JoinWallResponse;
    using ros_message_type = rj_msgs::msg::JoinWallResponse;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination.response_uid = source.response_uid;
        destination.robot_id = source.robot_id;
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = strategy::communication::JoinWallResponse{
            source.response_uid, source.robot_id};
    }
};


}  // namespace rclcpp