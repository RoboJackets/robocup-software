#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_msgs/msg/acknowledge.hpp>

namespace strategy::communication {

struct Acknowledge {
    uint32_t response_uid;
};

bool operator==(const Acknowledge& a, const Acknowledge& b);
void generate_uid(Acknowledge& response);

}  // namespace strategy::communication

namespace rclcpp {

template <>
struct TypeAdapter<strategy::communication::Acknowledge, rj_msgs::msg::Acknowledge> {
    using is_specialized = std::true_type;
    using custom_type = strategy::communication::Acknowledge;
    using ros_message_type = rj_msgs::msg::Acknowledge;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination.response_uid = source.response_uid;
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = strategy::communication::Acknowledge{source.response_uid};
    }
};


}  // namespace rclcpp