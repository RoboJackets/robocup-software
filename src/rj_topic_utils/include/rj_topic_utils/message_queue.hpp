#pragma once

#include <queue>
#include <rclcpp/rclcpp.hpp>

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
class MessageQueue {
public:
    MessageQueue() {
        static_assert(Policy == MessagePolicy::kQueue ||
                      Policy == MessagePolicy::kLatest);
    }
};

// ============================================================================
/**
 * @brief Partially specialized template of MessageQueue for
 * MessagePolicy::kQueue.
 * @tparam T The message type to use.
 * @tparam queue_size How large the queue should be.
 */
template <typename T>
class MessageQueue<T, MessagePolicy::kQueue, kUnboundedQueueSize> {
public:
    using SharedPtr = std::shared_ptr<
        MessageQueue<T, MessagePolicy::kQueue, kUnboundedQueueSize>>;
    /**
     * @brief Constructor for MessageQueue.
     * @param node The node.
     * @param topic Topic name to subscribe to.
     * @param qos_queue_size What queue size to use for QoS.
     */
    MessageQueue(rclcpp::Node* node, const std::string& topic,
                 size_t qos_queue_size = 10,
                 const rclcpp::SubscriptionOptions& subscription_options = {});

    /**
     * @brief Inserts all of the messages in the queue into the passed in
     * vector.
     * @param vector
     * @return Whether at least one item was added to the vector.
     */
    bool get_all(std::vector<std::unique_ptr<T>>& vector);

    /**
     * @brief Inserts all of the messages in the queue into the passed in
     * vector. Multithread safe version that uses a mutex.
     * @param vector
     * @return Whether at least one item was added to the vector.
     */
    bool get_all_threaded(std::vector<std::unique_ptr<T>>& vector);

    /**
     * @brief Returns the first message in the queue, if any.
     * @param ptr
     * @return Whether an item was popped off the queue and into the passed in
     * ptr.
     */
    bool get(std::unique_ptr<T>& ptr);

    [[nodiscard]] int size() const { return queue_.size(); }

private:
    rclcpp::Node* node_;
    typename rclcpp::Subscription<T>::SharedPtr sub_;
    std::deque<std::unique_ptr<T>> queue_;
    std::mutex queue_mutex_;
};

// ============================================================================
template <typename T>
MessageQueue<T, MessagePolicy::kQueue, kUnboundedQueueSize>::MessageQueue(
    rclcpp::Node* node, const std::string& topic, size_t qos_queue_size,
    const rclcpp::SubscriptionOptions& subscription_options)
    : node_{node} {
    const auto callback = [this](typename T::UniquePtr msg) {
        std::lock_guard<std::mutex> guard(queue_mutex_);
        queue_.emplace_back(std::move(msg));
    };
    sub_ = node->create_subscription<T>(topic, rclcpp::QoS{qos_queue_size},
                                        callback, subscription_options);
}

// ============================================================================
template <typename T>
bool MessageQueue<T, MessagePolicy::kQueue, kUnboundedQueueSize>::get_all(
    std::vector<std::unique_ptr<T>>& vector) {
    if (queue_.empty()) {
        return false;
    }

    vector.insert(vector.end(), std::make_move_iterator(queue_.begin()),
                  std::make_move_iterator(queue_.end()));
    queue_.clear();
    return true;
}

// ============================================================================
template <typename T>
bool MessageQueue<T, MessagePolicy::kQueue, kUnboundedQueueSize>::
    get_all_threaded(std::vector<std::unique_ptr<T>>& vector) {
    std::lock_guard<std::mutex> guard(queue_mutex_);
    if (queue_.empty()) {
        return false;
    }

    vector.insert(vector.end(), std::make_move_iterator(queue_.begin()),
                  std::make_move_iterator(queue_.end()));
    queue_.clear();
    return true;
}

// ============================================================================
template <typename T>
bool MessageQueue<T, MessagePolicy::kQueue, kUnboundedQueueSize>::get(
    std::unique_ptr<T>& ptr) {
    if (queue_.empty()) {
        return false;
    }
    ptr = std::move(queue_.front());
    queue_.pop_front();
}

}  // namespace rj_topic_utils