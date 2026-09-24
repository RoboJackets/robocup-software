#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/seeker_request.hpp"

namespace strategy::communication {

struct SeekerRequest {
    uint32_t request_uid;
    uint8_t robot_id;
    double seeking_point_x;
    double seeking_point_y;
    bool adding;
};

bool operator==(const SeekerRequest& a, const SeekerRequest& b);
void generate_uid(SeekerRequest& request);

}  // namespace strategy::communication

namespace rclcpp {

template <>
struct TypeAdapter<strategy::communication::SeekerRequest, rj_msgs::msg::SeekerRequest> {
    using is_specialized = std::true_type;
    using custom_type = strategy::communication::SeekerRequest;
    using ros_message_type = rj_msgs::msg::SeekerRequest;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination.request_uid = source.request_uid;
        destination.robot_id = source.robot_id;
        destination.seeking_point_x = source.seeking_point_x;
        destination.seeking_point_y = source.seeking_point_y;
        destination.adding = source.adding;
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = strategy::communication::SeekerRequest{
            source.request_uid, source.robot_id, source.seeking_point_x,
            source.seeking_point_y, source.adding};
    }
};


}  // namespace rclcpp