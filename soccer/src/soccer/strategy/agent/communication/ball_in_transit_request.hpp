#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>

#include "rj_msgs/msg/ball_in_transit_request.hpp"

namespace strategy::communication {

struct BallInTransitRequest {
    uint32_t request_uid;
    uint8_t from_robot_id;
};

bool operator==(const BallInTransitRequest& a, const BallInTransitRequest& b);
void generate_uid(BallInTransitRequest& request);

}  // namespace strategy::communication

namespace rj_convert {

template <>
struct RosConverter<strategy::communication::BallInTransitRequest,
                    rj_msgs::msg::BallInTransitRequest> {
    static rj_msgs::msg::BallInTransitRequest to_ros(
        const strategy::communication::BallInTransitRequest& from) {
        rj_msgs::msg::BallInTransitRequest result;
        result.request_uid = from.request_uid;
        result.from_robot_id = from.from_robot_id;
        return result;
    }

    static strategy::communication::BallInTransitRequest from_ros(
        const rj_msgs::msg::BallInTransitRequest& from) {
        return strategy::communication::BallInTransitRequest{
            from.request_uid,
            from.from_robot_id,
        };
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::BallInTransitRequest,
                  rj_msgs::msg::BallInTransitRequest);

}  // namespace rj_convert