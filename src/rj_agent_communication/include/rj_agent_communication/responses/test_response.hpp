#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_agent_msgs/msg/test_response.hpp"

namespace communication {

struct TestResponse {
    uint32_t response_uid;
    std::string message;
};

//NOLINTNEXTLINE(readability-identifier-length)
bool operator==(const TestResponse& a, const TestResponse& b);
void generate_uid(TestResponse& response);

}  // namespace communication

namespace rj_convert {

template <>
struct RosConverter<communication::TestResponse, rj_agent_msgs::msg::TestResponse> {
    static rj_agent_msgs::msg::TestResponse to_ros(const communication::TestResponse& from) {
        rj_agent_msgs::msg::TestResponse result;
        result.response_uid = from.response_uid;
        result.message = from.message;
        return result;
    }

    static communication::TestResponse from_ros(const rj_agent_msgs::msg::TestResponse& from) {
        return communication::TestResponse{
            from.response_uid,
            from.message,
        };
    }
};

ASSOCIATE_CPP_ROS(communication::TestResponse, rj_agent_msgs::msg::TestResponse);

}  // namespace rj_convert