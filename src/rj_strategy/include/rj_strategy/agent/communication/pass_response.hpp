#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/pass_response.hpp"

namespace strategy::communication {

struct PassResponse {
    uint32_t response_uid;
    bool direct_open;
};

bool operator==(const PassResponse& a, const PassResponse& b);
void generate_uid(PassResponse& response);

}  // namespace strategy::communication

namespace rclcpp {

template <>
struct TypeAdapter<strategy::communication::PassResponse, rj_msgs::msg::PassResponse> {
    using is_specialized = std::true_type;
    using custom_type = strategy::communication::PassResponse;
    using ros_message_type = rj_msgs::msg::PassResponse;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination.response_uid = source.response_uid;
        destination.direct_open = source.direct_open;
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = strategy::communication::PassResponse{
            source.response_uid, source.direct_open};
    }
};


}  // namespace rclcpp