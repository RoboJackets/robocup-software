#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>

#include "rj_msgs/msg/acknowledge.hpp"

namespace strategy::communication {

struct Acknowledge {
    uint32_t response_uid;
};

bool operator==(const Acknowledge& a, const Acknowledge& b);
void generate_uid(Acknowledge& response);

}  // namespace strategy::communication

namespace rj_convert {

template <>
struct RosConverter<strategy::communication::Acknowledge, rj_msgs::msg::Acknowledge> {
    static rj_msgs::msg::Acknowledge to_ros(const strategy::communication::Acknowledge& from) {
        rj_msgs::msg::Acknowledge result;
        result.response_uid = from.response_uid;
        return result;
    }

    static strategy::communication::Acknowledge from_ros(const rj_msgs::msg::Acknowledge& from) {
        return strategy::communication::Acknowledge{
            from.response_uid,
        };
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::Acknowledge, rj_msgs::msg::Acknowledge);

}  // namespace rj_convert