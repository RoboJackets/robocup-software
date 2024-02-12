#pragma once 

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/scorer_cancel_request.hpp"

namespace strategy::communication {

struct ScorerCancelRequest {
	uint32_t request_uid;
	uint8_t robot_id;
};

bool operator==(const ScorerCancelRequest& a, const ScorerCancelRequest& b);
void generate_uid(ScorerCancelRequest& request);

}

namespace rj_convert {

template <>
struct RosConverter<strategy::communication::ScorerCancelRequest, rj_msgs::msg::ScorerCancelRequest> {
	static rj_msgs::msg::ScorerCancelRequest to_ros(const strategy::communication::ScorerCancelRequest& from) {
		rj_msgs::msg::ScorerCancelRequest result;
		result.request_uid = from.request_uid;
		result.robot_id = from.robot_id;
		return result;
	}

	static strategy::communication::ScorerCancelRequest from_ros(const rj_msgs::msg::ScorerCancelRequest& from) {
		return strategy::communication::ScorerCancelRequest{
			from.request_uid,
			from.robot_id,
		};
	}

};

ASSOCIATE_CPP_ROS(strategy::communication::ScorerCancelRequest, rj_msgs::msg::ScorerCancelRequest);

}