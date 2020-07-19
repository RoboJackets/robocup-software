#pragma once
namespace rj_topic_utils {
// ============================================================================
template <typename T>
MessageQueueNode<T, MessagePolicy::kQueue, kUnboundedQueueSize>::
    MessageQueueNode(const std::string& name, const std::string& topic,
                     size_t qos_queue_size)
    : rclcpp::Node(name) {
    const auto callback = [this](typename T::UniquePtr msg) {
        std::lock_guard<std::mutex> guard(queue_mutex_);
        queue_.emplace_back(std::move(msg));
    };
    sub_ = create_subscription<T>(topic, rclcpp::QoS{qos_queue_size}, callback);
}

// ============================================================================
template <typename T>
bool MessageQueueNode<T, MessagePolicy::kQueue, kUnboundedQueueSize>::GetAll(
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
bool MessageQueueNode<T, MessagePolicy::kQueue, kUnboundedQueueSize>::
    GetAllThreaded(std::vector<std::unique_ptr<T>>& vector) {
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
bool MessageQueueNode<T, MessagePolicy::kQueue, kUnboundedQueueSize>::Get(
    std::unique_ptr<T>& ptr) {
    if (queue_.empty()) {
        return false;
    }
    ptr = std::move(queue_.front());
    queue_.pop_front();
}

// ============================================================================
template <typename T>
MessageQueueNode<T, MessagePolicy::kQueue, 1>::MessageQueueNode(
    const std::string& name, const std::string& topic)
    : rclcpp::Node(name) {
    const auto callback = [this](typename T::UniquePtr msg) {
        std::lock_guard<std::mutex> latest_guard(latest_mutex_);
        latest_ = std::move(msg);
    };
    sub_ = create_subscription<T>(topic, rclcpp::QoS{1}, callback);
}

// ============================================================================
template <typename T>
std::unique_ptr<T> MessageQueueNode<T, MessagePolicy::kQueue, 1>::Get() {
    return std::move(latest_);
}

// ============================================================================
template <typename T>
std::unique_ptr<T>
MessageQueueNode<T, MessagePolicy::kQueue, 1>::GetThreaded() {
    std::lock_guard<std::mutex> latest_guard(latest_mutex_);
    return std::move(latest_);
}

// ============================================================================
template <typename T>
MessageQueueNode<T, MessagePolicy::kLatest>::MessageQueueNode(
    const std::string& name, const std::string& topic,
    std::shared_ptr<T> default_value)
    : rclcpp::Node(name), latest_{std::move(default_value)} {
    const auto callback = [this](typename T::SharedPtr msg) {
        std::lock_guard<std::mutex> latest_guard(latest_mutex_);
        latest_ = std::move(msg);
    };
    sub_ = create_subscription<T>(topic, rclcpp::QoS{1}, callback);
}

// ============================================================================
template <typename T>
std::shared_ptr<T> MessageQueueNode<T, MessagePolicy::kLatest>::Get() {
    return latest_;
}

// ============================================================================
template <typename T>
std::shared_ptr<T> MessageQueueNode<T, MessagePolicy::kLatest>::GetThreaded() {
    std::lock_guard<std::mutex> latest_guard(latest_mutex_);
    return latest_;
}

}  // namespace rj_topic_utils