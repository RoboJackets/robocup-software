#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/latency.hpp>

class RegistryPublisher {
public:
    // Singleton pattern
    static RegistryPublisher* getInstance() {
        if (instance == nullptr) {
            instance = new RegistryPublisher();
        }

        return instance;
    }

    void publish(std::string label, std::int8_t robot_id, uint64_t time);

private:
    static RegistryPublisher* instance;

    // Private Constructor
    RegistryPublisher();

    // Delete Copy Constructor and Assignment
    RegistryPublisher(const RegistryPublisher& other) = delete;
    RegistryPublisher& operator=(const RegistryPublisher& other) = delete;

    std::shared_ptr<rclcpp::Node> node_ =
        std::make_shared<rclcpp::Node>("rj_benchmarking_publisher");
    rclcpp::Publisher<rj_msgs::msg::Latency>::SharedPtr publisher_ =
        node_->create_publisher<rj_msgs::msg::Latency>("/registry", 100);
};
