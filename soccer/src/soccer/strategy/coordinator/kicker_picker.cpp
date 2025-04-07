#include "kicker_picker.hpp"

#include <algorithm>  // for std::any_of
#include <limits>

#include <rj_constants/topic_names.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_msgs/msg/kicker_picker.hpp>

namespace strategy {

KickerPicker::KickerPicker()
    : Coordinator("kicker_picker_srv", "kicker_picker_data", "kicker_picker_node") {
    // Subscribe to world state
    world_state_sub_ = this->create_subscription<rj_msgs::msg::WorldState>(
        vision_filter::topics::kWorldStateTopic, rclcpp::QoS(1),
        [this](rj_msgs::msg::WorldState::SharedPtr world_state) {  // NOLINT
            last_world_state_ = rj_convert::convert_from_ros(*world_state);
            publish_selected_kicker();
        });
}

void KickerPicker::service_callback(RequestPtr request, ResponsePtr response) {
    bool membership_changed = wants_to_kick_by_id_[request->robot_id] != request->wants_to_kick;

    wants_to_kick_by_id_[request->robot_id] = request->wants_to_kick;

    if (membership_changed) {
        // Potential concern: this slows down the callback. Will agents be busy-waiting on a
        // response?
        publish_selected_kicker();
    }

    response->success = true;
}

void KickerPicker::publish_selected_kicker() {
    // Find closest robot to ball among group members
    double min_distance = std::numeric_limits<double>::infinity();
    uint8_t selected_kicker = kInvalidRobotId;

    const auto& ball_pos = last_world_state_.ball.position;

    for (uint8_t i = 0; i < kNumShells; ++i) {
        if (wants_to_kick_by_id_[i]) {
            const auto& robot = last_world_state_.get_robot(true, i);
            double distance = ball_pos.dist_to(robot.pose.position());
            if (distance < min_distance) {
                min_distance = distance;
                selected_kicker = i;
            }
        }
    }

    // Only publish if the selected kicker has changed
    if (selected_kicker != last_published_kicker_) {
        publisher_->publish(rj_msgs::msg::KickerPicker().set__robot_id(selected_kicker));
        last_published_kicker_ = selected_kicker;
    }
}

}  // namespace strategy
