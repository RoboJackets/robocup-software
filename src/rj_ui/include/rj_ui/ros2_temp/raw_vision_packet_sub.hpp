#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_common/context.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_msgs/msg/detail/raw_protobuf__struct.hpp>
#include <rj_msgs/msg/raw_protobuf.hpp>
#include <rj_topic_utils/async_message_queue.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>

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

    std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> ros_executor_;
    using RawProtobufMsg = rj_msgs::msg::RawProtobuf;
    rclcpp::Node::SharedPtr raw_protobuf_node_;
    rclcpp::Subscription<rj_msgs::msg::RawProtobuf>::SharedPtr raw_protobuf_sub_;

    using RawProtobufMsgQueue = rj_topic_utils::AsyncMessageQueue<
        RawProtobufMsg, rj_topic_utils::MessagePolicy::kQueue>;
    RawProtobufMsgQueue::UniquePtr queue_;
};
}  // namespace ros2_temp