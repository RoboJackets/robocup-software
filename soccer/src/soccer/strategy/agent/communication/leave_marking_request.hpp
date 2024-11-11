#pragma once 

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/leave_marking_request.hpp"

namespace strategy::communication {

struct LeaveMarkingRequest {
	uint32_t request_uid;
	uint8_t robot_id;
	uint8_t marked_robot_id;
};

bool operator==(const LeaveMarkingRequest& a, const LeaveMarkingRequest& b);
void generate_uid(LeaveMarkingRequest& request);

}

namespace rj_convert {

template <>
struct RosConverter<strategy::communication::LeaveMarkingRequest, rj_msgs::msg::LeaveMarkingRequest> {
	static rj_msgs::msg::LeaveMarkingRequest to_ros(const strategy::communication::LeaveMarkingRequest& from) {
		rj_msgs::msg::LeaveMarkingRequest result;
		result.request_uid = from.request_uid;
		result.robot_id = from.robot_id;
		result.marked_robot_id = from.marked_robot_id;
		return result;
	}

	static strategy::communication::LeaveMarkingRequest from_ros(const rj_msgs::msg::LeaveMarkingRequest& from) {
		return strategy::communication::LeaveMarkingRequest{
			from.request_uid,
			from.robot_id,
			from.marked_robot_id,
		};
	}

};

ASSOCIATE_CPP_ROS(strategy::communication::LeaveMarkingRequest, rj_msgs::msg::LeaveMarkingRequest);

}