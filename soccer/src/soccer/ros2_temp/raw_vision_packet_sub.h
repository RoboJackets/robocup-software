#pragma once

#include <ros2_temp/async_message_queue.h>

#include <Context.hpp>
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
    RawVisionPacketSub(Context* context);

    /**
     * @brief Updates context->raw_vision_packets with the raw protobuf message
     * from vision_receiver.
     */
    void run();

private:
    Context* context_;

    using RawProtobufMsgQueue =
        AsyncMessageQueue<RawProtobufMsg, MessagePolicy::QUEUE>;
    RawProtobufMsgQueue::SharedPtr queue_;
};
}  // namespace ros2_temp