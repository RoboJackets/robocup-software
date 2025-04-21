#pragma once

#include <array>

#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/marking.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <rj_msgs/srv/marking.hpp>

#include "coordinator.hpp"
#include "rj_constants/constants.hpp"
#include "world_state.hpp"

namespace strategy {

class Marking
    : public Coordinator<Marking, rj_msgs::srv::Marking, rj_msgs::msg::Marking> {
public:
    static constexpr uint8_t kInvalidRobotId = kNumShells;

    Marking();
    ~Marking() override = default;
    Marking(const Marking&) = delete;
    Marking& operator=(const Marking&) = delete;
    Marking(Marking&&) = delete;
    Marking& operator=(Marking&&) = delete;

    void service_callback(RequestPtr request, ResponsePtr response);

private:
    void publish_marking_list();

    std::array<int, kNumShells> marking_list{};  // TODO: Initialize it to invalid robot id
    std::array<bool, kNumShells> valid_targets{}; // 0 initialized, no one is a valid target initially
    WorldState last_world_state_;
    rclcpp::Subscription<rj_msgs::msg::WorldState>::SharedPtr world_state_sub_;
};

}  // namespace strategy