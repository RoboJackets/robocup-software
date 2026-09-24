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

namespace rclcpp {

template <>
struct TypeAdapter<strategy::communication::ScorerRequest, rj_msgs::msg::ScorerRequest> {
    using is_specialized = std::true_type;
    using custom_type = strategy::communication::ScorerRequest;
    using ros_message_type = rj_msgs::msg::ScorerRequest;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination.request_uid = source.request_uid;
        destination.robot_id = source.robot_id;
        destination.ball_distance = source.ball_distance;
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = strategy::communication::ScorerRequest{
            source.request_uid, source.robot_id, source.ball_distance};
    }
};


}  // namespace rclcpp