#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>

#include "rj_msgs/msg/kicker_request.hpp"

namespace strategy::communication {

struct KickerRequest {
    uint32_t request_uid;
    uint8_t robot_id;
    double distance;
};

bool operator==(const KickerRequest& a, const KickerRequest& b);
void generate_uid(KickerRequest& request);

}  // namespace strategy::communication

namespace rj_convert {

template <>
struct RosConverter<strategy::communication::KickerRequest, rj_msgs::msg::KickerRequest> {
    static rj_msgs::msg::KickerRequest to_ros(const strategy::communication::KickerRequest& from) {
        rj_msgs::msg::KickerRequest result;
        result.request_uid = from.request_uid;
        result.robot_id = from.robot_id;
        result.distance = from.distance;
        return result;
    }

    static strategy::communication::KickerRequest from_ros(
        const rj_msgs::msg::KickerRequest& from) {
        return strategy::communication::KickerRequest{
            from.request_uid,
            from.robot_id,
            from.distance,
        };
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::KickerRequest, rj_msgs::msg::KickerRequest);

}  // namespace rj_convert