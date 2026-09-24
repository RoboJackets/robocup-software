#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/position_request.hpp"

namespace strategy::communication {

struct PositionRequest {
    uint32_t request_uid;
};

bool operator==(const PositionRequest& a, const PositionRequest& b);
void generate_uid(PositionRequest& request);

}  // namespace strategy::communication

namespace rclcpp {

template <>
struct TypeAdapter<strategy::communication::PositionRequest, rj_msgs::msg::PositionRequest> {
    using is_specialized = std::true_type;
    using custom_type = strategy::communication::PositionRequest;
    using ros_message_type = rj_msgs::msg::PositionRequest;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination.request_uid = source.request_uid;
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = strategy::communication::PositionRequest{source.request_uid};
    }
};


}  // namespace rclcpp