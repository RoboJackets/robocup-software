#pragma once
namespace rj_topic_utils {
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
bool MessageQueue<T, MessagePolicy::kQueue, kUnboundedQueueSize>::GetAll(
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
bool MessageQueue<T, MessagePolicy::kQueue, kUnboundedQueueSize>::Get(
    std::unique_ptr<T>& ptr) {
    if (queue_.empty()) {
        return false;
    }
    ptr = std::move(queue_.front());
    queue_.pop_front();
}

// ============================================================================
template <typename T>
MessageQueue<T, MessagePolicy::kQueue, 1>::MessageQueue(
    rclcpp::Node* node, const std::string& topic,
    const rclcpp::SubscriptionOptions& subscription_options)
    : node_{node} {
    const auto callback = [this](typename T::UniquePtr msg) {
        std::lock_guard<std::mutex> latest_guard(latest_mutex_);
        latest_ = std::move(msg);
    };
    sub_ = node->create_subscription<T>(topic, rclcpp::QoS{1}, callback);
}

// ============================================================================
template <typename T>
std::unique_ptr<T> MessageQueue<T, MessagePolicy::kQueue, 1>::Get() {
    return std::move(latest_);
}

// ============================================================================
template <typename T>
std::unique_ptr<T> MessageQueue<T, MessagePolicy::kQueue, 1>::GetThreaded() {
    std::lock_guard<std::mutex> latest_guard(latest_mutex_);
    return std::move(latest_);
}

// ============================================================================
template <typename T>
MessageQueue<T, MessagePolicy::kLatest>::MessageQueue(
    rclcpp::Node* node, const std::string& topic, const T& default_value,
    const rclcpp::SubscriptionOptions& subscription_options)
    : node_{node} {
    latest_ = std::make_shared<T>();
    *latest_ = default_value;

    const auto callback = [this](typename T::SharedPtr msg) {
        std::lock_guard<std::mutex> latest_guard(latest_mutex_);
        latest_ = std::move(msg);
    };
    sub_ = node_->create_subscription<T>(topic, rclcpp::QoS{1}, callback,
                                         subscription_options);
}

// ============================================================================
template <typename T>
MessageQueue<T, MessagePolicy::kLatest>::MessageQueue(
    rclcpp::Node* node, const std::string& topic,
    const rclcpp::SubscriptionOptions& subscription_options)
    : node_{node} {
    const auto callback = [this](typename T::SharedPtr msg) {
        std::lock_guard<std::mutex> latest_guard(latest_mutex_);
        latest_ = std::move(msg);
    };
    sub_ = node_->create_subscription<T>(topic, rclcpp::QoS{1}, callback,
                                         subscription_options);
}

// ============================================================================
template <typename T>
std::shared_ptr<T> MessageQueue<T, MessagePolicy::kLatest>::Get() {
    return latest_;
}

// ============================================================================
template <typename T>
std::shared_ptr<T> MessageQueue<T, MessagePolicy::kLatest>::GetThreaded() {
    std::lock_guard<std::mutex> latest_guard(latest_mutex_);
    return latest_;
}

}  // namespace rj_topic_utils