#pragma once

#include <array>
#include <rclcpp/rclcpp.hpp>
#include <rj_msgs/msg/kicker_picker.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <rj_msgs/srv/kicker_picker.hpp>
#include "coordinator.hpp"
#include "rj_constants/constants.hpp"
#include "world_state.hpp"

namespace strategy {

class KickerPicker : public Coordinator<KickerPicker, rj_msgs::srv::KickerPicker, rj_msgs::msg::KickerPicker> {
public:
    static constexpr uint8_t kInvalidRobotId = kNumShells;

    KickerPicker();
    ~KickerPicker() override = default;
    KickerPicker(const KickerPicker&) = delete;
    KickerPicker& operator=(const KickerPicker&) = delete;
    KickerPicker(KickerPicker&&) = delete;
    KickerPicker& operator=(KickerPicker&&) = delete;

    void service_callback(RequestPtr request, ResponsePtr response);

private:
    void publish_selected_kicker();

    std::array<bool, kNumShells> wants_to_kick_by_id_ {};  // Zero-initialized
    WorldState last_world_state_;
    rclcpp::Subscription<rj_msgs::msg::WorldState>::SharedPtr world_state_sub_;
    uint8_t last_published_kicker_ = kInvalidRobotId;  // Track last published kicker
};

}  // namespace strategy