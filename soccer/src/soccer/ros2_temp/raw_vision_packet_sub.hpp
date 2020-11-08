#pragma once

#include <rj_topic_utils/async_message_queue.hpp>

#include <context.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rj_msgs/msg/raw_protobuf.hpp>

namespace ros2_temp {
using RawProtobufMsg = rj_msgs::msg::RawProtobuf;

/**
 * @brief A temporary class (until the logging framework is ported) that obtains
 * the raw protobuf messages from vision_receiver by spinning off a thread to
 * handle callbacks.
 */
class RawVisionPacketSub {
public:
    using UniquePtr = std::unique_ptr<RawVisionPacketSub>;
    RawVisionPacketSub(Context* context);

    /**
     * @brief Updates context->raw_vision_packets with the raw protobuf message
     * from vision_receiver.
     */
    void run();

private:
    Context* context_;

    using RawProtobufMsgQueue = rj_topic_utils::AsyncMessageQueue<
        RawProtobufMsg, rj_topic_utils::MessagePolicy::kQueue>;
    RawProtobufMsgQueue::UniquePtr queue_;
};
}  // namespace ros2_temp