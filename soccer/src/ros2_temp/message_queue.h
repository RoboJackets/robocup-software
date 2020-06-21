#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <rj_utils/logging.hpp>

namespace ros2_temp {
/**
 * @brief A temporary node that acts as a message queue for messages.
 * @tparam T The message type this collects.
 */
template <typename T>
class MessageQueueNode : public rclcpp::Node {
public:
    /**
     * @brief Constructor for MessageQueueNode.
     * @param name
     * @param topic
     * @param qos
     */
    MessageQueueNode(const std::string& name, const std::string& topic,
                     const rclcpp::QoS& qos = rclcpp::QoS{5});

    /**
     * @brief Inserts all of the messages in the queue into the passed in
     * vector.
     * @param vector
     * @return
     */
    bool GetAll(std::vector<std::unique_ptr<T>>& vector);

    /**
     * @brief Inserts all of the messages in the queue into the passed in
     * vector. Multithread safe version that uses a mutex.
     * @param vector
     * @return
     */
    bool GetAllThreaded(std::vector<std::unique_ptr<T>>& vector);

    /**
     * @brief Returns the first message in the queue, if any.
     * @param ptr
     * @return
     */
    bool Get(std::unique_ptr<T>& ptr);

    [[nodiscard]] int size() const { return queue_.size(); }

private:
    typename rclcpp::Subscription<T>::SharedPtr sub_;
    std::deque<std::unique_ptr<T>> queue_;
    std::mutex queue_mutex_;
};

template <typename T>
MessageQueueNode<T>::MessageQueueNode(const std::string& name,
                                      const std::string& topic,
                                      const rclcpp::QoS& qos)
    : rclcpp::Node(name) {
    const auto callback = [this](typename T::UniquePtr msg) {
        std::lock_guard<std::mutex> guard(queue_mutex_);
        queue_.emplace_back(std::move(msg));
    };
    sub_ = create_subscription<T>(topic, qos, callback);
}

template <typename T>
bool MessageQueueNode<T>::GetAll(std::vector<std::unique_ptr<T>>& vector) {
    if (queue_.empty()) {
        return false;
    }

    vector.insert(vector.end(), std::make_move_iterator(queue_.begin()),
                  std::make_move_iterator(queue_.end()));
    queue_.clear();
    return true;
}

template <typename T>
bool MessageQueueNode<T>::GetAllThreaded(
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
bool MessageQueueNode<T>::Get(std::unique_ptr<T>& ptr) {
    if (queue_.empty()) {
        return false;
    }
    ptr = std::move(queue_.front());
    queue_.pop_front();
}

}  // namespace ros2_temp
