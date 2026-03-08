#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_agent_msgs/msg/acknowledge.hpp>

namespace communication {

struct Acknowledge {
    uint32_t response_uid;
};

//NOLINTNEXTLINE(readability-identifier-length)
bool operator==(const Acknowledge& a, const Acknowledge& b);
void generate_uid(Acknowledge& response);

}  // namespace communication

namespace rj_convert {

template <>
struct RosConverter<communication::Acknowledge, rj_agent_msgs::msg::Acknowledge> {
    static rj_agent_msgs::msg::Acknowledge to_ros(const communication::Acknowledge& from) {
        rj_agent_msgs::msg::Acknowledge result;
        result.response_uid = from.response_uid;
        return result;
    }

    static communication::Acknowledge from_ros(const rj_agent_msgs::msg::Acknowledge& from) {
        return communication::Acknowledge{
            from.response_uid,
        };
    }
};

ASSOCIATE_CPP_ROS(communication::Acknowledge, rj_agent_msgs::msg::Acknowledge);

}  // namespace rj_convert