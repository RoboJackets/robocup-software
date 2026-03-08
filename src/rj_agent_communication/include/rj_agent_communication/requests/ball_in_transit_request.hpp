#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_agent_msgs/msg/ball_in_transit_request.hpp>

namespace communication {

struct BallInTransitRequest {
    uint32_t request_uid;
    uint8_t from_robot_id;
};

//NOLINTNEXTLINE(readability-identifier-length)
bool operator==(const BallInTransitRequest& a, const BallInTransitRequest& b);
void generate_uid(BallInTransitRequest& request);

}  // namespace communication

namespace rj_convert {

template <>
struct RosConverter<communication::BallInTransitRequest,
                    rj_agent_msgs::msg::BallInTransitRequest> {
    static rj_agent_msgs::msg::BallInTransitRequest to_ros(
        const communication::BallInTransitRequest& from) {
        rj_agent_msgs::msg::BallInTransitRequest result;
        result.request_uid = from.request_uid;
        result.from_robot_id = from.from_robot_id;
        return result;
    }

    static communication::BallInTransitRequest from_ros(
        const rj_agent_msgs::msg::BallInTransitRequest& from) {
        return communication::BallInTransitRequest{
            from.request_uid,
            from.from_robot_id,
        };
    }
};

ASSOCIATE_CPP_ROS(communication::BallInTransitRequest,
                  rj_agent_msgs::msg::BallInTransitRequest);

}  // namespace rj_convert