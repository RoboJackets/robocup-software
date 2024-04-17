#pragma once 

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/pass_request.hpp"

namespace strategy::communication {

struct PassRequest {
	uint32_t request_uid;
	bool direct;
	uint8_t from_robot_id;
};

bool operator==(const PassRequest& a, const PassRequest& b);
void generate_uid(PassRequest& request);

}

namespace rj_convert {

template <>
struct RosConverter<strategy::communication::PassRequest, rj_msgs::msg::PassRequest> {
	static rj_msgs::msg::PassRequest to_ros(const strategy::communication::PassRequest& from) {
		rj_msgs::msg::PassRequest result;
		result.request_uid = from.request_uid;
		result.direct = from.direct;
		result.from_robot_id = from.from_robot_id;
		return result;
	}

	static strategy::communication::PassRequest from_ros(const rj_msgs::msg::PassRequest& from) {
		return strategy::communication::PassRequest{
			from.request_uid,
			from.direct,
			from.from_robot_id,
		};
	}

};

ASSOCIATE_CPP_ROS(strategy::communication::PassRequest, rj_msgs::msg::PassRequest);

}