#include "rj_benchmarking/registry_publisher.hpp"

RegistryPublisher* RegistryPublisher::instance = nullptr;

RegistryPublisher::RegistryPublisher()
{
    printf("Testing");
}

void RegistryPublisher::publish(std::string label, std::int8_t robot_id, uint64_t time)
{
    // idk what type this should be im just following tutorial code
    auto message = rj_msgs::msg::Latency();
    
    message.label = label;
    message.robot_id = robot_id;
    message.duration_ns = time;

    publisher_->publish(message);
}
