#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_agent_msgs/msg/test_request.hpp"

namespace communication {

struct TestRequest {
    uint32_t request_uid;
};

//NOLINTNEXTLINE(readability-identifier-length)
bool operator==(const TestRequest& a, const TestRequest& b);
void generate_uid(TestRequest& request);

}  // namespace communication

namespace rj_convert {

template <>
struct RosConverter<communication::TestRequest, rj_agent_msgs::msg::TestRequest> {
    static rj_agent_msgs::msg::TestRequest to_ros(const communication::TestRequest& from) {
        rj_agent_msgs::msg::TestRequest result;
        result.request_uid = from.request_uid;
        return result;
    }

    static communication::TestRequest from_ros(const rj_agent_msgs::msg::TestRequest& from) {
        return communication::TestRequest{
            from.request_uid,
        };
    }
};

ASSOCIATE_CPP_ROS(communication::TestRequest, rj_agent_msgs::msg::TestRequest);

}  // namespace rj_convert