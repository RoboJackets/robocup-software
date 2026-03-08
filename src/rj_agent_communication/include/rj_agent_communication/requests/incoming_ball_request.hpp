#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_agent_msgs/msg/incoming_ball_request.hpp"

namespace communication {

struct IncomingBallRequest {
    uint32_t request_uid;
    uint8_t from_robot_id;
};

//NOLINTNEXTLINE(readability-identifier-length)
bool operator==(const IncomingBallRequest& a, const IncomingBallRequest& b);
void generate_uid(IncomingBallRequest& request);

}  // namespace communication

namespace rj_convert {

template <>
struct RosConverter<communication::IncomingBallRequest,
                    rj_agent_msgs::msg::IncomingBallRequest> {
    static rj_agent_msgs::msg::IncomingBallRequest to_ros(
        const communication::IncomingBallRequest& from) {
        rj_agent_msgs::msg::IncomingBallRequest result;
        result.request_uid = from.request_uid;
        result.from_robot_id = from.from_robot_id;
        return result;
    }

    static communication::IncomingBallRequest from_ros(
        const rj_agent_msgs::msg::IncomingBallRequest& from) {
        return communication::IncomingBallRequest{
            from.request_uid,
            from.from_robot_id,
        };
    }
};

ASSOCIATE_CPP_ROS(communication::IncomingBallRequest, rj_agent_msgs::msg::IncomingBallRequest);

}  // namespace rj_convert