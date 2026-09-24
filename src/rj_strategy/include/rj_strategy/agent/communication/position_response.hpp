#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/position_response.hpp"

namespace strategy::communication {

struct PositionResponse {
    uint32_t response_uid;
    std::string position;
};

bool operator==(const PositionResponse& a, const PositionResponse& b);
void generate_uid(PositionResponse& response);

}  // namespace strategy::communication

namespace rclcpp {

template <>
struct TypeAdapter<strategy::communication::PositionResponse, rj_msgs::msg::PositionResponse> {
    using is_specialized = std::true_type;
    using custom_type = strategy::communication::PositionResponse;
    using ros_message_type = rj_msgs::msg::PositionResponse;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination.response_uid = source.response_uid;
        destination.position = source.position;
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = strategy::communication::PositionResponse{
            source.response_uid, source.position};
    }
};


}  // namespace rclcpp