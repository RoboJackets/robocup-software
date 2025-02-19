#pragma once 

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/leave_marking_response.hpp"

namespace strategy::communication {

struct LeaveMarkingResponse {
	uint32_t response_uid;
	uint8_t robot_id;
	uint8_t marked_robot_id;
};

bool operator==(const LeaveMarkingResponse& a, const LeaveMarkingResponse& b);
void generate_uid(LeaveMarkingResponse& response);

}

namespace rj_convert {

template <>
struct RosConverter<strategy::communication::LeaveMarkingResponse, rj_msgs::msg::LeaveMarkingResponse> {
	static rj_msgs::msg::LeaveMarkingResponse to_ros(const strategy::communication::LeaveMarkingResponse& from) {
		rj_msgs::msg::LeaveMarkingResponse result;
		result.response_uid = from.response_uid;
		result.robot_id = from.robot_id;
		result.marked_robot_id = from.marked_robot_id;
		return result;
	}

	static strategy::communication::LeaveMarkingResponse from_ros(const rj_msgs::msg::LeaveMarkingResponse& from) {
		return strategy::communication::LeaveMarkingResponse{
			from.response_uid,
			from.robot_id,
			from.marked_robot_id,
		};
	}

};

ASSOCIATE_CPP_ROS(strategy::communication::LeaveMarkingResponse, rj_msgs::msg::LeaveMarkingResponse);

}