namespace rj_topic_utils {
// ============================================================================
template <typename T, int queue_size>
AsyncMessageQueue<T, MessagePolicy::kQueue, queue_size>::AsyncMessageQueue(
    const std::string& node_name, const std::string& topic_name) {
    queue_ = std::make_shared<
        MessageQueueNode<T, MessagePolicy::kQueue, queue_size>>(node_name,
                                                                topic_name);
    executor_.add_node(queue_);
    worker_ = std::thread([this]() { executor_.spin(); });
}

// ============================================================================
template <typename T, int queue_size>
std::vector<std::unique_ptr<T>>
AsyncMessageQueue<T, MessagePolicy::kQueue, queue_size>::GetAll() {
    std::vector<std::unique_ptr<T>> vec;
    queue_->GetAllThreaded(vec);
    return vec;
}

// ============================================================================
template <typename T>
AsyncMessageQueue<T, MessagePolicy::kQueue, 1>::AsyncMessageQueue(
    const std::string& node_name, const std::string& topic_name) {
    queue_ = std::make_shared<MessageQueueNode<T, MessagePolicy::kQueue, 1>>(
        node_name, topic_name);
    executor_.add_node(queue_);
    worker_ = std::thread([this]() { executor_.spin(); });
}

// ============================================================================
template <typename T>
std::unique_ptr<T> AsyncMessageQueue<T, MessagePolicy::kQueue, 1>::Get() {
    return queue_->GetThreaded();
}

// ============================================================================
template <typename T>
AsyncMessageQueue<T, MessagePolicy::kLatest>::AsyncMessageQueue(
    const std::string& node_name, const std::string& topic_name,
    std::shared_ptr<T> default_value) {
    queue_ = std::make_shared<MessageQueueNode<T, MessagePolicy::kLatest>>(
        node_name, topic_name, std::move(default_value));
    executor_.add_node(queue_);
    worker_ = std::thread([this]() { executor_.spin(); });
}

// ============================================================================
template <typename T>
std::shared_ptr<T> AsyncMessageQueue<T, MessagePolicy::kLatest>::Get() {
    return queue_->GetThreaded();
}
}  // namespace rj_topic_utils