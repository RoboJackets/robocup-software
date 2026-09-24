#pragma once

#include <rj_protos/ssl_vision_wrapper.pb.h>
#include <rj_protos/ssl_vision_wrapper.pb.h>
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

namespace rclcpp {

template<>
struct TypeAdapter<SSL_DetectionRobot, DetectionRobotMsg> {
    using is_specialized = std::true_type;
	using custom_type = SSL_DetectionRobot;
	using ros_message_type = DetectionRobotMsg;

	static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
	    destination.confidence = source.confidence();
	    destination.robot_id = source.robot_id();
	    destination.x = source.x();
	    destination.y = source.y();
	    destination.orientation = source.orientation();
	    destination.pixel_x = source.pixel_x();
	    destination.pixel_y = source.pixel_y();
	    destination.height = source.height();
	}

	static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
	    destination.set_confidence(source.confidence);
	    destination.set_robot_id(source.robot_id);
	    destination.set_x(source.x);
	    destination.set_y(source.y);
	    destination.set_orientation(source.orientation);
	    destination.set_pixel_x(source.pixel_x);
	    destination.set_pixel_y(source.pixel_y);
	    destination.set_height(source.height);
	}
};

template <>
struct TypeAdapter<SSL_DetectionBall, DetectionBallMsg> {
    using is_specialized = std::true_type;
    using custom_type = SSL_DetectionBall;
    using ros_message_type = DetectionBallMsg;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination.confidence = source.confidence();
        destination.area = source.area();
        destination.x = source.x();
        destination.y = source.y();
        destination.z = source.z();
        destination.pixel_x = source.pixel_x();
        destination.pixel_y = source.pixel_y();
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination.set_confidence(source.confidence);
        destination.set_area(source.area);
        destination.set_x(source.x);
        destination.set_y(source.y);
        destination.set_z(source.z);
        destination.set_pixel_x(source.pixel_x);
        destination.set_pixel_y(source.pixel_y);
    }
};

}  // namespace rclcpp
