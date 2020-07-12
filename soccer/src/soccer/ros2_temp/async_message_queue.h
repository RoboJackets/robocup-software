#pragma once
#include <ros2_temp/message_queue.h>

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

namespace ros2_temp {

// TODO(?): Extract this out into a proper ros2 topics library.

/**
 * @brief Declared but not defined template class, so that Policy must be one
 * of MessagePolicy::kQueue or MessagePolicy::kLatest.
 * @tparam T
 * @tparam Policy
 */
template <typename T, MessagePolicy Policy>
class AsyncMessageQueue;

/**
 * @brief A asynchronoous message queue that stores messages from a ROS2 topic
 * into a queue, spinning off a worker thread to handle all the ROS2
 * subscription work.
 * @tparam T The message type to use.
 */
template <typename T>
class AsyncMessageQueue<T, MessagePolicy::kQueue> {
public:
    using UniquePtr =
        std::unique_ptr<AsyncMessageQueue<T, MessagePolicy::kQueue>>;

    AsyncMessageQueue(const std::string& node_name,
                      const std::string& topic_name, int queue_size = 5);

    /**
     * @brief Returns a vector of unique_ptr to the received ROS2 messages,
     * emptying the internal queue.
     * @return A vector of all the messages in in chronologically
     * ascending order (first is oldest, last is newest).
     */
    std::vector<std::unique_ptr<T>> GetAll();

private:
    std::shared_ptr<MessageQueueNode<T, MessagePolicy::kQueue>> queue_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread worker_;
};

/**
 * @brief A asynchronoous message queue that stores only the latest messages
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
                      const std::string& topic_name);

    /**
     * @brief Returns a unique_ptr to the latest received message, or nullptr
     * if none have been received so far.
     * @return unique_ptr to the latest received message, or nullptr
     * if none have been received so far.
     */
    std::unique_ptr<T> Get();

private:
    std::shared_ptr<MessageQueueNode<T, MessagePolicy::kLatest>> queue_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread worker_;
};

template <typename T>
AsyncMessageQueue<T, MessagePolicy::kQueue>::AsyncMessageQueue(
    const std::string& node_name, const std::string& topic_name,
    const int queue_size) {
    queue_ = std::make_shared<MessageQueueNode<T, MessagePolicy::kQueue>>(
        node_name, topic_name, queue_size);
    executor_.add_node(queue_);
    worker_ = std::thread([this]() { executor_.spin(); });
}

template <typename T>
std::vector<std::unique_ptr<T>>
AsyncMessageQueue<T, MessagePolicy::kQueue>::GetAll() {
    std::vector<std::unique_ptr<T>> vec;
    queue_->GetAllThreaded(vec);
    return vec;
}

template <typename T>
AsyncMessageQueue<T, MessagePolicy::kLatest>::AsyncMessageQueue(
    const std::string& node_name, const std::string& topic_name) {
    queue_ = std::make_shared<MessageQueueNode<T, MessagePolicy::kLatest>>(
        node_name, topic_name);
    executor_.add_node(queue_);
    worker_ = std::thread([this]() { executor_.spin(); });
}

template <typename T>
std::unique_ptr<T> AsyncMessageQueue<T, MessagePolicy::kLatest>::Get() {
    return queue_->Get();
}

}  // namespace ros2_temp
