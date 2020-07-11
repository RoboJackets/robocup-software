#pragma once

#include <ros2_temp/message_queue.h>

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
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::shared_ptr<MessageQueueNode<RawProtobufMsg>> queue_;
    std::thread worker_;

    /**
     * @brief Calls executor_.spin().
     */
    void spinForever();
};
}  // namespace ros2_temp