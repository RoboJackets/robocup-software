#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/pass_request.hpp"

namespace strategy::communication {

struct PassRequest {
    uint32_t request_uid;
    bool direct;
    uint8_t from_robot_id;
};

bool operator==(const PassRequest& a, const PassRequest& b);
void generate_uid(PassRequest& request);

}  // namespace strategy::communication

namespace rclcpp {

template <>
struct TypeAdapter<strategy::communication::PassRequest, rj_msgs::msg::PassRequest> {
    using is_specialized = std::true_type;
    using custom_type = strategy::communication::PassRequest;
    using ros_message_type = rj_msgs::msg::PassRequest;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination.request_uid = source.request_uid;
        destination.direct = source.direct;
        destination.from_robot_id = source.from_robot_id;
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = strategy::communication::PassRequest{
            source.request_uid, source.direct, source.from_robot_id};
    }
};


}  // namespace rclcpp