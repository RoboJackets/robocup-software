#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/scorer_response.hpp"

namespace strategy::communication {

struct ScorerResponse {
    uint32_t response_uid;
    uint8_t robot_id;
    double ball_distance;
};

bool operator==(const ScorerResponse& a, const ScorerResponse& b);
void generate_uid(ScorerResponse& response);

}  // namespace strategy::communication

namespace rj_convert {

template <>
struct RosConverter<strategy::communication::ScorerResponse, rj_msgs::msg::ScorerResponse> {
    static rj_msgs::msg::ScorerResponse to_ros(
        const strategy::communication::ScorerResponse& from) {
        rj_msgs::msg::ScorerResponse result;
        result.response_uid = from.response_uid;
        result.robot_id = from.robot_id;
        result.ball_distance = from.ball_distance;
        return result;
    }

    static strategy::communication::ScorerResponse from_ros(
        const rj_msgs::msg::ScorerResponse& from) {
        return strategy::communication::ScorerResponse{
            from.response_uid,
            from.robot_id,
            from.ball_distance,
        };
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::ScorerResponse, rj_msgs::msg::ScorerResponse);

}  // namespace rj_convert