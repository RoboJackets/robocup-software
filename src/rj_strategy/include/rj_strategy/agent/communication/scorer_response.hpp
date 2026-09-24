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

namespace rclcpp {

template <>
struct TypeAdapter<strategy::communication::ScorerResponse, rj_msgs::msg::ScorerResponse> {
    using is_specialized = std::true_type;
    using custom_type = strategy::communication::ScorerResponse;
    using ros_message_type = rj_msgs::msg::ScorerResponse;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination.response_uid = source.response_uid;
        destination.robot_id = source.robot_id;
        destination.ball_distance = source.ball_distance;
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = strategy::communication::ScorerResponse{
            source.response_uid, source.robot_id, source.ball_distance};
    }
};


}  // namespace rclcpp