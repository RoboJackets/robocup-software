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

namespace rclcpp {

template <>
struct TypeAdapter<strategy::communication::ResetScorerRequest, rj_msgs::msg::ResetScorerRequest> {
    using is_specialized = std::true_type;
    using custom_type = strategy::communication::ResetScorerRequest;
    using ros_message_type = rj_msgs::msg::ResetScorerRequest;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination.request_uid = source.request_uid;
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = strategy::communication::ResetScorerRequest{source.request_uid};
    }
};


}  // namespace rclcpp