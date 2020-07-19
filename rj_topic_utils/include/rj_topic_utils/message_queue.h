#pragma once

#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <rj_utils/logging.hpp>

namespace rj_topic_utils {
// ============================================================================
/**
 * @brief What kind of policy to use for the queue.
 *
 * kLatest only stores the latest message.
 * kQueue stores a queue of the last N messages.
 */
enum class MessagePolicy { kQueue, kLatest };

constexpr int kUnboundedQueueSize = -1;

/**
 * @brief A temporary node that acts as a message queue for messages.
 * @tparam T The message type this collects.
 * @tparam Policy What policy to use for the queue.
 * @tparam queue_size How large should the queue be.
 */
template <typename T, MessagePolicy Policy,
          int queue_size = kUnboundedQueueSize>
class MessageQueueNode : public rclcpp::Node {
public:
    MessageQueueNode() {
        static_assert(Policy == MessagePolicy::kQueue ||
                      Policy == MessagePolicy::kLatest);
    }
};

// ============================================================================
/**
 * @brief Partially specialized template of MessageQueueNode for
 * MessagePolicy::kQueue.
 * @tparam T The message type to use.
 * @tparam queue_size How large the queue should be.
 */
template <typename T>
class MessageQueueNode<T, MessagePolicy::kQueue, kUnboundedQueueSize>
    : public rclcpp::Node {
public:
    using SharedPtr = std::shared_ptr<
        MessageQueueNode<T, MessagePolicy::kQueue, kUnboundedQueueSize>>;
    /**
     * @brief Constructor for MessageQueueNode.
     * @param name Name of the node.
     * @param topic Topic name to subscribe to.
     * @param qos_queue_size What queue size to use for QoS.
     */
    MessageQueueNode(const std::string& name, const std::string& topic,
                     size_t qos_queue_size = 10);

    /**
     * @brief Inserts all of the messages in the queue into the passed in
     * vector.
     * @param vector
     * @return Whether at least one item was added to the vector.
     */
    bool GetAll(std::vector<std::unique_ptr<T>>& vector);

    /**
     * @brief Inserts all of the messages in the queue into the passed in
     * vector. Multithread safe version that uses a mutex.
     * @param vector
     * @return Whether at least one item was added to the vector.
     */
    bool GetAllThreaded(std::vector<std::unique_ptr<T>>& vector);

    /**
     * @brief Returns the first message in the queue, if any.
     * @param ptr
     * @return Whether an item was popped off the queue and into the passed in
     * ptr.
     */
    bool Get(std::unique_ptr<T>& ptr);

    [[nodiscard]] int size() const { return queue_.size(); }

private:
    typename rclcpp::Subscription<T>::SharedPtr sub_;
    std::deque<std::unique_ptr<T>> queue_;
    std::mutex queue_mutex_;
};

// ============================================================================
/**
 * @brief Partially specialized template of MessageQueueNode for
 * MessagePolicy::kQueue, when queue size is 1.
 * @tparam T The message type to use.
 */
template <typename T>
class MessageQueueNode<T, MessagePolicy::kQueue, 1> : public rclcpp::Node {
public:
    using SharedPtr =
        std::shared_ptr<MessageQueueNode<T, MessagePolicy::kQueue, 1>>;
    /**
     * @brief Constructor for MessageQueueNode.
     * @param name
     * @param topic
     */
    MessageQueueNode(const std::string& name, const std::string& topic);

    /**
     * @brief Returns a unique_ptr to item in the queue, emptying the queue. If
     * the queue is empty, returns nullptr.
     * @return unique_ptr to the item in the queue, returning nullptr if the
     * queue is empty.
     */
    std::unique_ptr<T> Get();

    /**
     * @brief Returns a unique_ptr to item in the queue, emptying the queue. If
     * the queue is empty, returns nullptr. Threadsafe version of the above
     * that locks a mutex.
     * @return unique_ptr to the item in the queue, returning nullptr if the
     * queue is empty.
     */
    std::unique_ptr<T> GetThreaded();

private:
    typename rclcpp::Subscription<T>::SharedPtr sub_;
    std::unique_ptr<T> latest_;
    std::mutex latest_mutex_;
};

// ============================================================================
/**
 * @brief Partially specialized template of MessageQueueNode for
 * MessagePolicy::kQueue.
 * @tparam T The message type to use.
 */
template <typename T>
class MessageQueueNode<T, MessagePolicy::kLatest> : public rclcpp::Node {
public:
    using SharedPtr = std::shared_ptr<MessageQueueNode>;
    /**
     * @brief Constructor for MessageQueueNode.
     * @param name Name of the node.
     * @param topic What topic to subscribe to.
     * @param default_value The default value to use.
     */
    MessageQueueNode(const std::string& name, const std::string& topic,
                     std::shared_ptr<T> default_value = nullptr);

    /**
     * @brief Returns the latest message in the queue if we have received one
     * so far, otherwise nullptr if no default is set.
     * @param ptr
     * @return The latest message if we have received one so far, otherwise
     * returns nullptr if no default is set.
     */
    std::shared_ptr<T> Get();

    /**
     * @brief Returns the latest message in the queue if we have received one
     * so far, otherwise nullptr if no default is set. Multithread safe version
     * of the above.
     * @param ptr
     * @return The latest message if we have received one so far, otherwise
     * returns nullptr if no default is set.
     */
    std::shared_ptr<T> GetThreaded();

private:
    typename rclcpp::Subscription<T>::SharedPtr sub_;
    std::shared_ptr<T> latest_;
    std::mutex latest_mutex_;
};

}  // namespace rj_topic_utils

#include "message_queue_impl.h"
