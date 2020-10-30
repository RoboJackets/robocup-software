#pragma once

#include <rclcpp/executors/single_threaded_executor.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rj {

class SpinAllExecutor : public rclcpp::executors::SingleThreadedExecutor {
public:
    void spin_all(std::chrono::nanoseconds max_duration);
};

}  // namespace rj
