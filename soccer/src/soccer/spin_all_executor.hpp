#pragma once

#include <rclcpp/rclcpp.hpp>

namespace rj {

class SpinAllExecutor : public rclcpp::SingleThreadedExecutor {
public:
    void spin_all();
};

}  // namespace rj
