#pragma once
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

#include "message_queue.hpp"

namespace rj_topic_utils {

/**
 * @brief Declared but not defined template class, so that Policy must be one
 * of MessagePolicy::kQueue or MessagePolicy::kLatest.
 * @tparam T The message type to use.
 * @tparam Policy What policy to use.
 * @tparam queue_size The queue size to use. For now, this can either be
 * kUnboundedQueueSize or 1.
 */
template <typename T, MessagePolicy Policy,
          int queue_size = kUnboundedQueueSize>
class AsyncMessageQueue;

// ============================================================================
/**
 * @brief A asynchronous message queue that stores messages from a ROS2 topic
 * into a queue, spinning off a worker thread to handle all the ROS2
 * subscription work.
 * @tparam T The message type to use.
 * @tparam queue_size
 */
template <typename T, int queue_size>
class AsyncMessageQueue<T, MessagePolicy::kQueue, queue_size> {
public:
    using UniquePtr = std::unique_ptr<
        AsyncMessageQueue<T, MessagePolicy::kQueue, queue_size>>;

    AsyncMessageQueue(const std::string& node_name,
                      const std::string& topic_name);

    /**
     * @brief Returns a vector of unique_ptr to the received ROS2 messages,
     * emptying the internal queue.
     * @return A vector of all the messages in in chronologically
     * ascending order (first is oldest, last is newest).
     */
    std::vector<std::unique_ptr<T>> get_all();

private:
    rclcpp::Node::SharedPtr node_;
    MessageQueue<T, MessagePolicy::kQueue> queue_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread worker_;
};

// ============================================================================
template <typename T, int queue_size>
AsyncMessageQueue<T, MessagePolicy::kQueue, queue_size>::AsyncMessageQueue(
    const std::string& node_name, const std::string& topic_name)
    : node_{rclcpp::Node::make_shared(node_name)},
      queue_{node_.get(), topic_name} {
    executor_.add_node(node_);
    worker_ = std::thread([this]() { executor_.spin(); });
}

// ============================================================================
template <typename T, int queue_size>
std::vector<std::unique_ptr<T>>
AsyncMessageQueue<T, MessagePolicy::kQueue, queue_size>::get_all() {
    std::vector<std::unique_ptr<T>> vec;
    queue_.get_all_threaded(vec);
    return vec;
}

}  // namespace rj_topic_utils
