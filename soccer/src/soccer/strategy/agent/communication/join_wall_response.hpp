#pragma once 

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/join_wall_response.hpp"

namespace strategy::communication {

struct JoinWallResponse {
	uint32_t response_uid;
	uint8_t robot_id;
};

bool operator==(const JoinWallResponse& a, const JoinWallResponse& b);
void generate_uid(JoinWallResponse& response);

}

namespace rj_convert {

template <>
struct RosConverter<strategy::communication::JoinWallResponse, rj_msgs::msg::JoinWallResponse> {
	static rj_msgs::msg::JoinWallResponse to_ros(const strategy::communication::JoinWallResponse& from) {
		rj_msgs::msg::JoinWallResponse result;
		result.response_uid = from.response_uid;
		result.robot_id = from.robot_id;
		return result;
	}

	static strategy::communication::JoinWallResponse from_ros(const rj_msgs::msg::JoinWallResponse& from) {
		return strategy::communication::JoinWallResponse{
			from.response_uid,
			from.robot_id,
		};
	}

};

ASSOCIATE_CPP_ROS(strategy::communication::JoinWallResponse, rj_msgs::msg::JoinWallResponse);

}