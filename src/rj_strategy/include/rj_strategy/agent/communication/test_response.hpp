#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/test_response.hpp"

namespace strategy::communication {

struct TestResponse {
    uint32_t response_uid;
    std::string message;
};

bool operator==(const TestResponse& a, const TestResponse& b);
void generate_uid(TestResponse& response);

}  // namespace strategy::communication

namespace rclcpp {

template <>
struct TypeAdapter<strategy::communication::TestResponse, rj_msgs::msg::TestResponse> {
    using is_specialized = std::true_type;
    using custom_type = strategy::communication::TestResponse;
    using ros_message_type = rj_msgs::msg::TestResponse;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination.response_uid = source.response_uid;
        destination.message = source.message;
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = strategy::communication::TestResponse{
            source.response_uid, source.message};
    }
};


}  // namespace rclcpp