#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/incoming_ball_request.hpp"

namespace strategy::communication {

struct IncomingBallRequest {
    uint32_t request_uid;
    uint8_t from_robot_id;
};

bool operator==(const IncomingBallRequest& a, const IncomingBallRequest& b);
void generate_uid(IncomingBallRequest& request);

}  // namespace strategy::communication

namespace rclcpp {

template <>
struct TypeAdapter<strategy::communication::IncomingBallRequest,
                    rj_msgs::msg::IncomingBallRequest> {
    using is_specialized = std::true_type;
    using custom_type = strategy::communication::IncomingBallRequest;
    using ros_message_type = rj_msgs::msg::IncomingBallRequest;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination.request_uid = source.request_uid;
        destination.from_robot_id = source.from_robot_id;
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = strategy::communication::IncomingBallRequest{
            source.request_uid, source.from_robot_id};
    }
};


}  // namespace rclcpp