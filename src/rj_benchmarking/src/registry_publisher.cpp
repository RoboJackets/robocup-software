#include "rj_benchmarking/registry_publisher.hpp"

RegistryPublisher* RegistryPublisher::instance = nullptr;

void RegistryPublisher::publish(const std::string& label, u_int8_t robot_id, uint64_t time) {
    rj_msgs::msg::Latency message = rj_msgs::msg::Latency();

    message.label = label;
    message.robot_id = robot_id;
    message.duration_ns = time;

    publisher_->publish(message);
}
