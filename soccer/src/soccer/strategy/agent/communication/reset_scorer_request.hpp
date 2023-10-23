#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>

#include "rj_msgs/msg/reset_scorer_request.hpp"

namespace strategy::communication {

struct ResetScorerRequest {
    uint32_t request_uid;
};

bool operator==(const ResetScorerRequest& a, const ResetScorerRequest& b);
void generate_uid(ResetScorerRequest& request);

}  // namespace strategy::communication

namespace rj_convert {

template <>
struct RosConverter<strategy::communication::ResetScorerRequest, rj_msgs::msg::ResetScorerRequest> {
    static rj_msgs::msg::ResetScorerRequest to_ros(
        const strategy::communication::ResetScorerRequest& from) {
        rj_msgs::msg::ResetScorerRequest result;
        result.request_uid = from.request_uid;
        return result;
    }

    static strategy::communication::ResetScorerRequest from_ros(
        const rj_msgs::msg::ResetScorerRequest& from) {
        return strategy::communication::ResetScorerRequest{
            from.request_uid,
        };
    }
};

ASSOCIATE_CPP_ROS(strategy::communication::ResetScorerRequest, rj_msgs::msg::ResetScorerRequest);

}  // namespace rj_convert