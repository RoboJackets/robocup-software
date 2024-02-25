#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/scorer_request.hpp"

namespace strategy::communication {

struct ScorerRequest {
    uint32_t request_uid;
    uint8_t robot_id;
    double ball_distance;
};

bool operator==(const ScorerRequest& a, const ScorerRequest& b);
void generate_uid(ScorerRequest& request);

}  // namespace strategy::communication

namespace rj_convert {

template <>
struct RosConverter<strategy::communication::ScorerRequest, rj_msgs::msg::ScorerRequest> {
    static rj_msgs::msg::ScorerRequest to_ros(const strategy::communication::ScorerRequest& from) {
        rj_msgs::msg::ScorerRequest result;
        result.request_uid = from.request_uid;
        result.robot_id = from.robot_id;
        result.ball_distance = from.ball_distance;
        return result;
    }

    static strategy::communication::ScorerRequest from_ros(
        const rj_msgs::msg::ScorerRequest& from) {
        return strategy::communication::ScorerRequest{
            from.request_uid,
            from.robot_id,
            from.ball_distance,
        };
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::ScorerRequest, rj_msgs::msg::ScorerRequest);

}  // namespace rj_convert