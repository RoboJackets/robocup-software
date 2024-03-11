#pragma once 

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/position_request.hpp"

namespace strategy::communication {

struct PositionRequest {
	uint32_t request_uid;
};

bool operator==(const PositionRequest& a, const PositionRequest& b);
void generate_uid(PositionRequest& request);

}

namespace rj_convert {

template <>
struct RosConverter<strategy::communication::PositionRequest, rj_msgs::msg::PositionRequest> {
	static rj_msgs::msg::PositionRequest to_ros(const strategy::communication::PositionRequest& from) {
		rj_msgs::msg::PositionRequest result;
		result.request_uid = from.request_uid;
		return result;
	}

	static strategy::communication::PositionRequest from_ros(const rj_msgs::msg::PositionRequest& from) {
		return strategy::communication::PositionRequest{
			from.request_uid,
		};
	}

};

ASSOCIATE_CPP_ROS(strategy::communication::PositionRequest, rj_msgs::msg::PositionRequest);

}