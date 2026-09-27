#include "rj_ui/ros2_temp/raw_vision_packet_sub.hpp"
#include <rj_constants/topic_names.hpp>

namespace ros2_temp {
RawVisionPacketSub::RawVisionPacketSub(Context* context) : context_{context} {
    queue_ = std::make_unique<RawProtobufMsgQueue>("raw_vision_packet_sub",
                                                   vision_receiver::topics::kRawProtobufTopic);
}

void RawVisionPacketSub::run() {
    ros_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    std::vector<RawProtobufMsg::UniquePtr> raw_protobufs = queue_->get_all();

    raw_protobuf_node_ = std::make_shared<rclcpp::Node>("_raw_protobuf_reciever");
    ros_executor_->add_node(raw_protobuf_node_, true);

    raw_protobuf_sub_ = raw_protobuf_node_->create_subscription<RawProtobufMsg>(
        vision_receiver::topics::kRawProtobufTopic, rclcpp::QoS(1),
        [this](RawProtobufMsg::UniquePtr msg) {
            // callback goes here
        })
    // Convert all RawProtobufMsgs to SSL_WrapperPacket
    for (const RawProtobufMsg::UniquePtr& msg : raw_protobufs) {
        context_->raw_vision_packets.emplace_back();
        context_->raw_vision_packets.back().ParseFromArray(msg->data.data(), msg->data.size());
    }
}

}  // namespace ros2_temp