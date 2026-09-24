#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_msgs/msg/ball_in_transit_request.hpp>

namespace strategy::communication {

struct BallInTransitRequest {
    uint32_t request_uid;
    uint8_t from_robot_id;
};

bool operator==(const BallInTransitRequest& a, const BallInTransitRequest& b);
void generate_uid(BallInTransitRequest& request);

}  // namespace strategy::communication

namespace rclcpp {

template <>
struct TypeAdapter<strategy::communication::BallInTransitRequest,
                    rj_msgs::msg::BallInTransitRequest> {
    using is_specialized = std::true_type;
    using custom_type = strategy::communication::BallInTransitRequest;
    using ros_message_type = rj_msgs::msg::BallInTransitRequest;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination.request_uid = source.request_uid;
        destination.from_robot_id = source.from_robot_id;
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = strategy::communication::BallInTransitRequest{
            source.request_uid, source.from_robot_id};
    }
};


}  // namespace rclcpp