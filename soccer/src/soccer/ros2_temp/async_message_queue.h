#pragma once
#include <ros2_temp/message_queue.h>

#include <rclcpp/rclcpp.hpp>
#include <thread>
#include <vector>

namespace ros2_temp {

/**
 * @brief A asynchronoous message queue that stores messages from a ROS2 topic
 * into a queue, spinning off a worker thread to handle all the ROS2
 * subscription work.
 */
template <typename T>
class AsyncMessageQueue {
public:
    AsyncMessageQueue(const std::string& node_name,
                      const std::string& topic_name);

    /**
     * @brief Returns a vector of the received DetectionFrameMsgs, emptying
     * queue_.
     * @return A vector of all the messages in queue_, in chronologically
     * ascending order (first is oldest, last is newest).
     */
    std::vector<std::unique_ptr<T>> GetAll();

private:
    std::shared_ptr<MessageQueueNode<T>> queue_;
    rclcpp::executors::SingleThreadedExecutor executor_;
    std::thread worker_;

    /**
     * Blocking calls executor_.spin(). Only exists when rclcpp::ok() returns
     * false.
     */
    void spin();
};

template <typename T>
AsyncMessageQueue<T>::AsyncMessageQueue(const std::string& node_name,
                                        const std::string& topic_name) {
    queue_ = std::make_shared<MessageQueueNode<T>>(node_name, topic_name);
    executor_.add_node(queue_);
    worker_ = std::thread(&AsyncMessageQueue::spin, this);
}

template <typename T>
std::vector<std::unique_ptr<T>> AsyncMessageQueue<T>::GetAll() {
    std::vector<std::unique_ptr<T>> vec;
    queue_->GetAllThreaded(vec);
    return vec;
}

template <typename T>
void AsyncMessageQueue<T>::spin() {
    executor_.spin();
}
}  // namespace ros2_temp
