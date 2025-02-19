#pragma once 

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/join_marking_response.hpp"

namespace strategy::communication {

struct JoinMarkingResponse {
	uint32_t response_uid;
	uint8_t robot_id;
	uint8_t marked_robot_id;
};

bool operator==(const JoinMarkingResponse& a, const JoinMarkingResponse& b);
void generate_uid(JoinMarkingResponse& response);

}

namespace rj_convert {

template <>
struct RosConverter<strategy::communication::JoinMarkingResponse, rj_msgs::msg::JoinMarkingResponse> {
	static rj_msgs::msg::JoinMarkingResponse to_ros(const strategy::communication::JoinMarkingResponse& from) {
		rj_msgs::msg::JoinMarkingResponse result;
		result.response_uid = from.response_uid;
		result.robot_id = from.robot_id;
		result.marked_robot_id = from.marked_robot_id;
		return result;
	}

	static strategy::communication::JoinMarkingResponse from_ros(const rj_msgs::msg::JoinMarkingResponse& from) {
		return strategy::communication::JoinMarkingResponse{
			from.response_uid,
			from.robot_id,
			from.marked_robot_id,
		};
	}

};

ASSOCIATE_CPP_ROS(strategy::communication::JoinMarkingResponse, rj_msgs::msg::JoinMarkingResponse);

}