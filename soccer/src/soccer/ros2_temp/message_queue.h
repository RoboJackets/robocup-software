#pragma once

#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <rj_utils/logging.hpp>

namespace ros2_temp {

/**
 * @brief What kind of policy to use for the queue.
 *
 * kLatest only stores the latest message.
 * kQueue stores a queue of the last N messages.
 */
enum class MessagePolicy { kQueue, kLatest };

/**
 * @brief A temporary node that acts as a message queue for messages.
 * @tparam T The message type this collects.
 * @tparam Policy What policy to use for the queue.
 */
template <typename T, MessagePolicy Policy>
class MessageQueueNode : public rclcpp::Node {
public:
    MessageQueueNode() {
        static_assert(Policy == MessagePolicy::kQueue ||
                      Policy == MessagePolicy::kLatest);
    }
};

/**
 * @brief Partially specialized template of MessageQueueNode for
 * MessagePolicy::kQueue.
 * @tparam T The message type to use.
 */
template <typename T>
class MessageQueueNode<T, MessagePolicy::kQueue> : public rclcpp::Node {
public:
    using SharedPtr = std::shared_ptr<MessageQueueNode>;
    /**
     * @brief Constructor for MessageQueueNode.
     * @param name
     * @param topic
     * @param queue_size
     */
    MessageQueueNode(const std::string& name, const std::string& topic,
                     size_t queue_size);

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
     * @param name
     * @param topic
     * @param queue_size
     */
    MessageQueueNode(const std::string& name, const std::string& topic);

    /**
     * @brief Returns the latest message in the queue if we have received cone
     * so far, otherwise nullptr.
     * @param ptr
     * @return The latest message if we have received one so far, otherwise
     * returns nullptr.
     */
    std::unique_ptr<T> Get();

    /**
     * @brief Returns 1 if we have received a message before, 0 otherwise.
     * This is NOT thread safe.
     * @return
     */
    [[nodiscard]] int size() const { return latest_ == nullptr ? 0 : 1; }

private:
    typename rclcpp::Subscription<T>::SharedPtr sub_;
    std::unique_ptr<T> latest_;
    std::mutex latest_mutex_;
};

template <typename T>
MessageQueueNode<T, MessagePolicy::kQueue>::MessageQueueNode(
    const std::string& name, const std::string& topic, size_t queue_size)
    : rclcpp::Node(name) {
    const auto callback = [this](typename T::UniquePtr msg) {
        std::lock_guard<std::mutex> guard(queue_mutex_);
        queue_.emplace_back(std::move(msg));
    };
    sub_ = create_subscription<T>(topic, rclcpp::QoS{queue_size}, callback);
}

template <typename T>
bool MessageQueueNode<T, MessagePolicy::kQueue>::GetAll(
    std::vector<std::unique_ptr<T>>& vector) {
    if (queue_.empty()) {
        return false;
    }

    vector.insert(vector.end(), std::make_move_iterator(queue_.begin()),
                  std::make_move_iterator(queue_.end()));
    queue_.clear();
    return true;
}

template <typename T>
bool MessageQueueNode<T, MessagePolicy::kQueue>::GetAllThreaded(
    std::vector<std::unique_ptr<T>>& vector) {
    std::lock_guard<std::mutex> guard(queue_mutex_);
    if (queue_.empty()) {
        return false;
    }

    vector.insert(vector.end(), std::make_move_iterator(queue_.begin()),
                  std::make_move_iterator(queue_.end()));
    queue_.clear();
    return true;
}

template <typename T>
bool MessageQueueNode<T, MessagePolicy::kQueue>::Get(std::unique_ptr<T>& ptr) {
    if (queue_.empty()) {
        return false;
    }
    ptr = std::move(queue_.front());
    queue_.pop_front();
}

template <typename T>
MessageQueueNode<T, MessagePolicy::kLatest>::MessageQueueNode(
    const std::string& name, const std::string& topic)
    : rclcpp::Node(name) {
    const auto callback = [this](typename T::UniquePtr msg) {
        std::lock_guard<std::mutex> latest_guard(latest_mutex_);
        latest_ = std::move(msg);
    };
    sub_ = create_subscription<T>(topic, rclcpp::QoS{1}, callback);
}

template <typename T>
std::unique_ptr<T> MessageQueueNode<T, MessagePolicy::kLatest>::Get() {
    std::lock_guard<std::mutex> latest_guard(latest_mutex_);
    return std::move(latest_);
}

}  // namespace ros2_temp
