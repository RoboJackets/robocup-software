#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/pass_received_request.hpp"

namespace strategy::communication {

struct PassReceivedRequest {
    uint32_t request_uid;
    uint8_t from_robot_id;
};

bool operator==(const PassReceivedRequest& a, const PassReceivedRequest& b);
void generate_uid(PassReceivedRequest& request);

}  // namespace strategy::communication

namespace rj_convert {

template <>
struct RosConverter<strategy::communication::PassReceivedRequest,
                    rj_msgs::msg::PassReceivedRequest> {
    static rj_msgs::msg::PassReceivedRequest to_ros(
        const strategy::communication::PassReceivedRequest& from) {
        rj_msgs::msg::PassReceivedRequest result;
        result.request_uid = from.request_uid;
        result.from_robot_id = from.from_robot_id;
        return result;
    }

    static strategy::communication::PassReceivedRequest from_ros(
        const rj_msgs::msg::PassReceivedRequest& from) {
        return strategy::communication::PassReceivedRequest{
            from.request_uid,
            from.from_robot_id,
        };
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::PassReceivedRequest,
                  rj_msgs::msg::PassReceivedRequest);

}  // namespace rj_convert
