#include <rj_constants/topic_names.hpp>
#include <ros2_temp/raw_vision_packet_sub.hpp>

namespace ros2_temp {
RawVisionPacketSub::RawVisionPacketSub(Context* context) : context_{context} {
    queue_ = std::make_unique<RawProtobufMsgQueue>("raw_vision_packet_sub",
                                                   vision_receiver::topics::kRawProtobufPub);
}

void RawVisionPacketSub::run() {
    std::vector<RawProtobufMsg::UniquePtr> raw_protobufs = queue_->get_all();

    // Convert all RawProtobufMsgs to SSL_WrapperPacket
    for (const RawProtobufMsg::UniquePtr& msg : raw_protobufs) {
        context_->raw_vision_packets.emplace_back();
        context_->raw_vision_packets.back().ParseFromArray(msg->data.data(), msg->data.size());
    }
}

}  // namespace ros2_temp
