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

namespace rj_convert {

template <>
struct RosConverter<strategy::communication::TestResponse, rj_msgs::msg::TestResponse> {
    static rj_msgs::msg::TestResponse to_ros(const strategy::communication::TestResponse& from) {
        rj_msgs::msg::TestResponse result;
        result.response_uid = from.response_uid;
        result.message = from.message;
        return result;
    }

    static strategy::communication::TestResponse from_ros(const rj_msgs::msg::TestResponse& from) {
        return strategy::communication::TestResponse{
            from.response_uid,
            from.message,
        };
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::TestResponse, rj_msgs::msg::TestResponse);

}  // namespace rj_convert