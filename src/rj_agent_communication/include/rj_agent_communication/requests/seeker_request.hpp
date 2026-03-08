#pragma once

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_agent_msgs/msg/seeker_request.hpp"

namespace communication {

struct SeekerRequest {
    uint32_t request_uid;
    uint8_t robot_id;
    double seeking_point_x;
    double seeking_point_y;
    bool adding;
};

//NOLINTNEXTLINE(readability-identifier-length)
bool operator==(const SeekerRequest& a, const SeekerRequest& b);
void generate_uid(SeekerRequest& request);

}  // namespace communication

namespace rj_convert {

template <>
struct RosConverter<communication::SeekerRequest, rj_agent_msgs::msg::SeekerRequest> {
    static rj_agent_msgs::msg::SeekerRequest to_ros(const communication::SeekerRequest& from) {
        rj_agent_msgs::msg::SeekerRequest result;
        result.request_uid = from.request_uid;
        result.robot_id = from.robot_id;
        result.seeking_point_x = from.seeking_point_x;
        result.seeking_point_y = from.seeking_point_y;
        result.adding = from.adding;
        return result;
    }

    static communication::SeekerRequest from_ros(
        const rj_agent_msgs::msg::SeekerRequest& from) {
        return communication::SeekerRequest{
            from.request_uid,     from.robot_id, from.seeking_point_x,
            from.seeking_point_y, from.adding,
        };
    }
};

ASSOCIATE_CPP_ROS(communication::SeekerRequest, rj_agent_msgs::msg::SeekerRequest);

}  // namespace rj_convert