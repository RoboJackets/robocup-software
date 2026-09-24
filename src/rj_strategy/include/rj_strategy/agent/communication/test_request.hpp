#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/test_request.hpp"

namespace strategy::communication {

struct TestRequest {
    uint32_t request_uid;
};

bool operator==(const TestRequest& a, const TestRequest& b);
void generate_uid(TestRequest& request);

}  // namespace strategy::communication

namespace rclcpp {

template <>
struct TypeAdapter<strategy::communication::TestRequest, rj_msgs::msg::TestRequest> {
    using is_specialized = std::true_type;
    using custom_type = strategy::communication::TestRequest;
    using ros_message_type = rj_msgs::msg::TestRequest;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination.request_uid = source.request_uid;
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = strategy::communication::TestRequest{source.request_uid};
    }
};


}  // namespace rclcpp