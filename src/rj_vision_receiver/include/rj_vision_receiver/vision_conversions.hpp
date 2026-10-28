#pragma once

#include <rclcpp/type_adapter.hpp>

#include <rj_msgs/msg/detection_ball.hpp>
#include <rj_msgs/msg/detection_frame.hpp>
#include <rj_msgs/msg/detection_robot.hpp>
#include <rj_msgs/msg/raw_protobuf.hpp>
#include <rj_protos/vision/ssl_vision_wrapper.pb.h>

template <>
struct rclcpp::TypeAdapter<SSL_DetectionRobot, rj_msgs::msg::DetectionRobot> {
    using is_specialized = std::true_type;
    using custom_type = SSL_DetectionRobot;
    using ros_message_type = rj_msgs::msg::DetectionRobot;

    static void convert_to_ros_message(const SSL_DetectionRobot& source,
                                       rj_msgs::msg::DetectionRobot& target) {
        target.confidence = source.confidence();
        target.robot_id = source.robot_id();
        target.x = source.x();
        target.y = source.y();
        target.orientation = source.orientation();
        target.pixel_x = source.pixel_x();
        target.pixel_y = source.pixel_y();
        target.height = source.height();
    }

    static void convert_to_custom(const rj_msgs::msg::DetectionRobot& source,
                                  SSL_DetectionRobot& target) {
        target.set_confidence(source.confidence);
        target.set_robot_id(source.robot_id);
        target.set_x(source.x);
        target.set_y(source.y);
        target.set_orientation(source.orientation);
        target.set_pixel_x(source.pixel_x);
        target.set_pixel_y(source.pixel_y);
        target.set_height(source.height);
    }
};

template <>
struct rclcpp::TypeAdapter<SSL_DetectionBall, rj_msgs::msg::DetectionBall> {
    using is_specialized = std::true_type;
    using custom_type = SSL_DetectionBall;
    using ros_message_type = rj_msgs::msg::DetectionBall;

    static void convert_to_ros_message(const SSL_DetectionBall& source,
                                       rj_msgs::msg::DetectionBall& target) {
        target.confidence = source.confidence();
        target.area = source.area();
        target.x = source.x();
        target.y = source.y();
        target.z = source.z();
        target.pixel_x = source.pixel_x();
        target.pixel_y = source.pixel_y();
    }

    static void convert_to_custom(const rj_msgs::msg::DetectionBall& source,
                                  SSL_DetectionBall& target) {
        target.set_confidence(source.confidence);
        target.set_area(source.area);
        target.set_x(source.x);
        target.set_y(source.y);
        target.set_z(source.z);
        target.set_pixel_x(source.pixel_x);
        target.set_pixel_y(source.pixel_y);
    }
};

template <>
struct rclcpp::TypeAdapter<SSL_WrapperPacket, rj_msgs::msg::RawProtobuf> {
    using is_specialized = std::true_type;
    using custom_type = SSL_WrapperPacket;
    using ros_message_type = rj_msgs::msg::RawProtobuf;

    static void convert_to_ros_message(const SSL_WrapperPacket& source,
                                       rj_msgs::msg::RawProtobuf& target) {
        target.data.resize(source.ByteSizeLong());
        source.SerializeWithCachedSizesToArray(target.data.data());
    }

    static void convert_to_custom(const rj_msgs::msg::RawProtobuf& source,
                                  SSL_WrapperPacket& target) {
        target.ParseFromArray(source.data.data(), source.data.size());
    }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(SSL_DetectionRobot, rj_msgs::msg::DetectionRobot);
RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(SSL_DetectionBall, rj_msgs::msg::DetectionBall);
RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(SSL_WrapperPacket, rj_msgs::msg::RawProtobuf);
