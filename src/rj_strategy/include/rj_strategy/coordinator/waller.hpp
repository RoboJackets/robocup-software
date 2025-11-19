#pragma once

#include <rclcpp/rclcpp.hpp>

#include <array>
#include <algorithm>

#include <rj_msgs/srv/waller.hpp>
#include <rj_msgs/msg/waller.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_common/world_state.hpp>
#include <rj_constants/constants.hpp>
#include <rj_convert/ros_convert.hpp>

#include "rj_strategy/coordinator.hpp"

namespace strategy {

class Waller
    : public Coordinator<Waller, rj_msgs::srv::Waller, rj_msgs::msg::Waller> {
public:
    Waller();
    ~Waller() override = default;
    Waller(const Waller&) = delete;
    Waller& operator=(const Waller&) = delete;
    Waller(Waller&&) = delete;
    Waller& operator=(Waller&&) = delete;

    void service_callback(RequestPtr request, ResponsePtr response);
private:
    std::array<u_int8_t, kNumShells> walling_robots_;
    u_int8_t num_wallers_ = 0;
    static constexpr int kMaxWallers = 4;

    void update_wallers();
};

}  // namespace strategy