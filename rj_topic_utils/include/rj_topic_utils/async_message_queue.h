#pragma once
#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

#include "message_queue.h"

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
/**
 * @brief A asynchronous message queue that stores messages from a ROS2 topic
 * into a queue, spinning off a worker thread to handle all the ROS2
 * subscription work.
 * @tparam T The message type to use.
 * @tparam queue_size The size of the queue.
 */
template <typename T>
class AsyncMessageQueue<T, MessagePolicy::kQueue, 1> {
public:
    using UniquePtr =
        std::unique_ptr<AsyncMessageQueue<T, MessagePolicy::kQueue, 1>>;

    AsyncMessageQueue(const std::string& node_name,
                      const std::string& topic_name);

    /**
     * @brief Returns a unique_ptr to item in the queue, emptying the queue. If
     * the queue is empty, returns nullptr.
     * @return unique_ptr to the item in the queue, returning nullptr if the
     * queue is empty.
     */
    std::unique_ptr<T> get();

private:
    rclcpp::Node::SharedPtr node_;
    MessageQueue<T, MessagePolicy::kQueue, 1> queue_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread worker_;
};

// ============================================================================
/**
 * @brief A asynchronous message queue that stores only the latest messages
 * from a ROS2 topic, spinning off a worker thread to handle all the ROS2
 * subscription work.
 * @tparam T The message type to use.
 */
template <typename T>
class AsyncMessageQueue<T, MessagePolicy::kLatest> {
public:
    using UniquePtr =
        std::unique_ptr<AsyncMessageQueue<T, MessagePolicy::kLatest>>;

    AsyncMessageQueue(const std::string& node_name,
                      const std::string& topic_name, const T& default_value);

    AsyncMessageQueue(const std::string& node_name,
                      const std::string& topic_name);

    /**
     * @brief Returns a shared_ptr to the latest received message, or nullptr
     * if none have been received so far.
     * @return shared_ptr to the latest received message, or nullptr
     * if none have been received so far.
     */
    std::shared_ptr<T> get();

private:
    rclcpp::Node::SharedPtr node_;
    MessageQueue<T, MessagePolicy::kLatest> queue_;
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

// ============================================================================
template <typename T>
AsyncMessageQueue<T, MessagePolicy::kQueue, 1>::AsyncMessageQueue(
    const std::string& node_name, const std::string& topic_name)
    : node_{rclcpp::Node::make_shared(node_name)},
      queue_{node_.get(), topic_name} {
    executor_.add_node(node_);
    worker_ = std::thread([this]() { executor_.spin(); });
}

// ============================================================================
template <typename T>
std::unique_ptr<T> AsyncMessageQueue<T, MessagePolicy::kQueue, 1>::get() {
    return queue_.get_threaded();
}

// ============================================================================
template <typename T>
AsyncMessageQueue<T, MessagePolicy::kLatest>::AsyncMessageQueue(
    const std::string& node_name, const std::string& topic_name,
    const T& default_value)
    : node_{rclcpp::Node::make_shared(node_name)},
      queue_{node_.get(), topic_name, default_value} {
    executor_.add_node(node_);
    worker_ = std::thread([this]() { executor_.spin(); });
}

// ============================================================================
template <typename T>
AsyncMessageQueue<T, MessagePolicy::kLatest>::AsyncMessageQueue(
    const std::string& node_name, const std::string& topic_name)
    : node_{rclcpp::Node::make_shared(node_name)},
      queue_{node_.get(), topic_name} {
    executor_.add_node(node_);
    worker_ = std::thread([this]() { executor_.spin(); });
}

// ============================================================================
template <typename T>
std::shared_ptr<T> AsyncMessageQueue<T, MessagePolicy::kLatest>::get() {
    return queue_.get_threaded();
}

}  // namespace rj_topic_utils
