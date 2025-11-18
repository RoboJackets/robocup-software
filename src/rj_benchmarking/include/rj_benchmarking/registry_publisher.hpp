#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/latency.hpp>

class RegistryPublisher {
public:
    // Singleton pattern
    static std::shared_ptr<RegistryPublisher> getInstance() {
        static std::shared_ptr<RegistryPublisher> instance{new RegistryPublisher()};
        return instance;
    }

    void publish(const std::string& label, uint8_t robot_id, uint64_t time);

private:
    static RegistryPublisher* instance;

    // Private Constructor
    RegistryPublisher() = default;

    // Delete Copy Constructor and Assignment
    RegistryPublisher(const RegistryPublisher& other) = delete;
    RegistryPublisher& operator=(const RegistryPublisher& other) = delete;

    std::shared_ptr<rclcpp::Node> node_ =
        std::make_shared<rclcpp::Node>("rj_benchmarking_publisher");
    rclcpp::Publisher<rj_msgs::msg::Latency>::SharedPtr publisher_ =
        node_->create_publisher<rj_msgs::msg::Latency>("/registry", 100);
};
