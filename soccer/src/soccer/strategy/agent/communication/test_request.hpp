#pragma once 

#include <mutex>
#include <string>
#include <variant>
#include <vector>

#include <rj_common/time.hpp>
#include <rj_convert/ros_convert.hpp>
#include "rj_msgs/msg/test_request.hpp"

namespace strategy::communication {

struct TestRequest {
	uint32_t request_uid;
};

bool operator==(const TestRequest& a, const TestRequest& b);
void generate_uid(TestRequest& request);

}

namespace rj_convert {

template <>
struct RosConverter<strategy::communication::TestRequest, rj_msgs::msg::TestRequest> {
	static rj_msgs::msg::TestRequest to_ros(const strategy::communication::TestRequest& from) {
		rj_msgs::msg::TestRequest result;
		result.request_uid = from.request_uid;
		return result;
	}

	static strategy::communication::TestRequest from_ros(const rj_msgs::msg::TestRequest& from) {
		return strategy::communication::TestRequest{
			from.request_uid,
		};
	}

};

ASSOCIATE_CPP_ROS(strategy::communication::TestRequest, rj_msgs::msg::TestRequest);

}