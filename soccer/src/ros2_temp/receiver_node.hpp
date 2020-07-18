#pragma once

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace ros2_temp {

template <typename T>
class ReceiverNode : public rclcpp::Node {
public:
    template <typename F>
    ReceiverNode(const std::string& name, const std::string& topic,
                 const F& message_callback,
                 const rclcpp::QoS qos = rclcpp::QoS{5})
        : rclcpp::Node(name) {
        subscription_ = create_subscription<T>(topic, qos, message_callback);
    }

private:
    typename rclcpp::Subscription<T>::SharedPtr subscription_;
};

}  // namespace ros2_temp