#pragma once 

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/join_marking_request.hpp"

namespace strategy::communication {

struct JoinMarkingRequest {
	uint32_t request_uid;
	uint8_t robot_id;
	uint8_t marked_robot_id;
};

bool operator==(const JoinMarkingRequest& a, const JoinMarkingRequest& b);
void generate_uid(JoinMarkingRequest& request);

}

namespace rj_convert {

template <>
struct RosConverter<strategy::communication::JoinMarkingRequest, rj_msgs::msg::JoinMarkingRequest> {
	static rj_msgs::msg::JoinMarkingRequest to_ros(const strategy::communication::JoinMarkingRequest& from) {
		rj_msgs::msg::JoinMarkingRequest result;
		result.request_uid = from.request_uid;
		result.robot_id = from.robot_id;
		result.marked_robot_id = from.marked_robot_id;
		return result;
	}

	static strategy::communication::JoinMarkingRequest from_ros(const rj_msgs::msg::JoinMarkingRequest& from) {
		return strategy::communication::JoinMarkingRequest{
			from.request_uid,
			from.robot_id,
			from.marked_robot_id,
		};
	}

};

ASSOCIATE_CPP_ROS(strategy::communication::JoinMarkingRequest, rj_msgs::msg::JoinMarkingRequest);

}