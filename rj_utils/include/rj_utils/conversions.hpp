#pragma once

#include <rj_protos/messages_robocup_ssl_wrapper.pb.h>

#include <rj_msgs/msg/detail/detection_ball__builder.hpp>
#include <rj_msgs/msg/detail/detection_robot__builder.hpp>
#include <rj_msgs/msg/detail/raw_protobuf__builder.hpp>

using RawProtobufMsg = rj_msgs::msg::RawProtobuf;
using DetectionBallMsg = rj_msgs::msg::DetectionBall;
using DetectionRobotMsg = rj_msgs::msg::DetectionRobot;

/**
 * @brief Converts from SSL_WrapperPacket to a RawProtobufMsg.
 * @param packet
 * @return
 */
[[nodiscard]] RawProtobufMsg::UniquePtr ToROSMsg(
    const SSL_WrapperPacket& packet);

/**
 * @brief Converts from SSL_DetectionBall to a DetectionBallMsg.
 * @param ball
 * @return
 */
[[nodiscard]] DetectionBallMsg ToROSMsg(const SSL_DetectionBall& ball);

/**
 * @brief Converts from SSL_DetectionRobot to a DetectionRobotMsg.
 * @param robot
 * @return
 */
[[nodiscard]] DetectionRobotMsg ToROSMsg(const SSL_DetectionRobot& robot);
