#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_agent_msgs/msg/scorer_request.hpp"

namespace communication {

struct ScorerRequest {
    uint32_t request_uid;
    uint8_t robot_id;
    double ball_distance;
};

//NOLINTNEXTLINE(readability-identifier-length)
bool operator==(const ScorerRequest& a, const ScorerRequest& b);
void generate_uid(ScorerRequest& request);

}  // namespace communication

namespace rj_convert {

template <>
struct RosConverter<communication::ScorerRequest, rj_agent_msgs::msg::ScorerRequest> {
    static rj_agent_msgs::msg::ScorerRequest to_ros(const communication::ScorerRequest& from) {
        rj_agent_msgs::msg::ScorerRequest result;
        result.request_uid = from.request_uid;
        result.robot_id = from.robot_id;
        result.ball_distance = from.ball_distance;
        return result;
    }

    static communication::ScorerRequest from_ros(
        const rj_agent_msgs::msg::ScorerRequest& from) {
        return communication::ScorerRequest{
            from.request_uid,
            from.robot_id,
            from.ball_distance,
        };
    }
};

ASSOCIATE_CPP_ROS(communication::ScorerRequest, rj_agent_msgs::msg::ScorerRequest);

}  // namespace rj_convert