#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_agent_msgs/msg/reset_scorer_request.hpp"

namespace communication {

struct ResetScorerRequest {
    uint32_t request_uid;
};

//NOLINTNEXTLINE(readability-identifier-length)
bool operator==(const ResetScorerRequest& a, const ResetScorerRequest& b);
void generate_uid(ResetScorerRequest& request);

}  // namespace communication

namespace rj_convert {

template <>
struct RosConverter<communication::ResetScorerRequest, rj_agent_msgs::msg::ResetScorerRequest> {
    static rj_agent_msgs::msg::ResetScorerRequest to_ros(
        const communication::ResetScorerRequest& from) {
        rj_agent_msgs::msg::ResetScorerRequest result;
        result.request_uid = from.request_uid;
        return result;
    }

    static communication::ResetScorerRequest from_ros(
        const rj_agent_msgs::msg::ResetScorerRequest& from) {
        return communication::ResetScorerRequest{
            from.request_uid,
        };
    }
};

ASSOCIATE_CPP_ROS(communication::ResetScorerRequest, rj_agent_msgs::msg::ResetScorerRequest);

}  // namespace rj_convert