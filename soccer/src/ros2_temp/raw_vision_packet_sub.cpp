#include "ros2_temp/raw_vision_packet_sub.h"

#include <rj_constants/topic_names.hpp>

namespace ros2_temp {
RawVisionPacketSub::RawVisionPacketSub(Context* context) : context_{context} {
    queue_ = std::make_shared<MessageQueueNode<RawProtobufMsg>>(
        "RawVisionPacketSub_queue", vision_receiver::topics::kRawProtobufPub);
    executor_.add_node(queue_);
    worker_ = std::thread{&RawVisionPacketSub::spinForever, this};
}

void RawVisionPacketSub::run() {
    std::vector<RawProtobufMsg::UniquePtr> raw_protobufs;
    if (!queue_->GetAllThreaded(raw_protobufs)) {
        return;
    }

    // Convert all RawProtobufMsgs to SSL_WrapperPacket
    for (const RawProtobufMsg::UniquePtr& msg : raw_protobufs) {
        context_->raw_vision_packets.emplace_back();
        context_->raw_vision_packets.back().ParseFromArray(msg->data.data(),
                                                           msg->data.size());
    }
}

void RawVisionPacketSub::spinForever() { executor_.spin(); }

}  // namespace ros2_temp
