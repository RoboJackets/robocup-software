#pragma once

#include <rj_protos/messages_robocup_ssl_wrapper.pb.h>
#include <rj_convert/ros_convert.hpp>

#include <rj_msgs/msg/detection_ball.hpp>
#include <rj_msgs/msg/detection_robot.hpp>
#include <rj_msgs/msg/raw_protobuf.hpp>

using RawProtobufMsg = rj_msgs::msg::RawProtobuf;
using DetectionBallMsg = rj_msgs::msg::DetectionBall;
using DetectionRobotMsg = rj_msgs::msg::DetectionRobot;

/**
 * @brief Converts from SSL_WrapperPacket to a RawProtobufMsg.
 * @param packet
 * @return
 */
[[nodiscard]] RawProtobufMsg::UniquePtr to_ros_msg(
    const SSL_WrapperPacket& packet);

/**
 * @brief Converts from SSL_DetectionBall to a DetectionBallMsg.
 * @param ball
 * @return
 */
[[nodiscard]] DetectionBallMsg to_ros_msg(const SSL_DetectionBall& ball);

/**
 * @brief Converts from SSL_DetectionRobot to a DetectionRobotMsg.
 * @param robot
 * @return
 */
[[nodiscard]] DetectionRobotMsg to_ros_msg(const SSL_DetectionRobot& robot);


namespace rj_convert {

template<>
struct RosConverter<SSL_DetectionRobot, RetectionRobotMsg> {
	static DetectionRobotMessage to_ros(const SSL_DetectionRobot& from) {
		DetectionRobotMsg to;
		
		convert_to_ros(from.confidence(), &to.confidence);
		convert_to_ros(from.robot_id(), &to.robot_id);
		convert_to_ros(from.x(), &to.x);
		convert_to_ros(from.y(), &to.y);
		convert_to_ros(from.orientation(), &to.orientation);
		convert_to_ros(from.pixel_x(), &to.pixel_x);
		convert_to_ros(from.pixel_y(), &to.pixel_y);
		convert_to_ros(from.height(), &to.height);
		
		return to;
	}
	
	static SSL_DetectionRobot from_ros(const DetectionRobotMsg& from) {
		SSL_DetectionRobot to;
		
		to.set_confidence(convert_from_ros(from.confidence));
		to.set_robot_id(convert_from_ros(from.robot_id));
		to.set_x(convert_from_ros(from.x));
		to.set_y(convert_from_ros(from.y));
		to.set_orientation(convert_from_ros(from.orientation));
		to.set_pixel_x(convert_from_ros(from.pixel_x));
		to.set_pixel_y(convert_from_ros(from.pixel_y));
		to.set_height(convert_from_ros(from.height));
		
		return to;
	}
};

ASSOCIATE_CPP_ROSS(SSL_DetectionRobot, DetectionRobotMsg)

}; // namespace rj_convert