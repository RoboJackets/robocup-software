#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rj_robocup/msg/raw_protobuf.hpp>
#include <rj_robocup/msg/vision_packet.hpp>

namespace receiver_nodes {
using RawProtobufMsg = rj_robocup::msg::RawProtobuf;
using VisionPacketMsg = rj_robocup::msg::VisionPacket;

/**
 * \brief This node subscribes to the VisionReceiver node and
 * puts it on a vector for Processor to consume
 */
class VisionReceiverSub : public rclcpp::Node {
 public:
  VisionReceiverSub();

  std::vector<RawProtobufMsg> raw_protobufs{};
  std::vector<VisionPacketMsg> vision_packets{};

 private:
  rclcpp::Subscription<RawProtobufMsg>::SharedPtr raw_vision_sub_;
  rclcpp::Subscription<VisionPacketMsg>::SharedPtr vision_packet_sub_;
};
}  // namespace receiver_nodes
