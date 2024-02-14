#pragma once 

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/incoming_ball_request.hpp"

namespace strategy::communication {

struct IncomingBallRequest {
	uint32_t request_uid;
	uint8_t from_robot_id;
};

bool operator==(const IncomingBallRequest& a, const IncomingBallRequest& b);
void generate_uid(IncomingBallRequest& request);

}

namespace rj_convert {

template <>
struct RosConverter<strategy::communication::IncomingBallRequest, rj_msgs::msg::IncomingBallRequest> {
	static rj_msgs::msg::IncomingBallRequest to_ros(const strategy::communication::IncomingBallRequest& from) {
		rj_msgs::msg::IncomingBallRequest result;
		result.request_uid = from.request_uid;
		result.from_robot_id = from.from_robot_id;
		return result;
	}

	static strategy::communication::IncomingBallRequest from_ros(const rj_msgs::msg::IncomingBallRequest& from) {
		return strategy::communication::IncomingBallRequest{
			from.request_uid,
			from.from_robot_id,
		};
	}

};

ASSOCIATE_CPP_ROS(strategy::communication::IncomingBallRequest, rj_msgs::msg::IncomingBallRequest);

}