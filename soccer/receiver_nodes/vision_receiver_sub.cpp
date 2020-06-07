#include <receiver_nodes/vision_receiver_sub.h>

namespace receiver_nodes {
VisionReceiverSub::VisionReceiverSub() : rclcpp::Node{"vision_receiver_sub"} {
  const auto raw_protobuf_cb = [this](RawProtobufMsg::UniquePtr msg) {
    raw_protobufs.emplace_back(*msg);
  };
  const auto vision_packet_cb = [this](VisionPacketMsg::UniquePtr msg) {
    vision_packets.emplace_back(*msg);
  };

  raw_vision_sub_ = create_subscription<RawProtobufMsg>("vision/raw_protobuf",
                                                        10, raw_protobuf_cb);
  vision_packet_sub_ = create_subscription<VisionPacketMsg>(
      "vision/vision_packet", 10, vision_packet_cb);
}
}  // namespace receiver_nodes
