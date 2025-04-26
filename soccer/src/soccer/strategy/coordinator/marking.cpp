#include "marking.hpp"

#include <algorithm>  // for std::any_of
#include <limits>

#include <rj_constants/topic_names.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_msgs/msg/marking.hpp>

namespace strategy {

Marking::Marking()
    : Coordinator("marking_srv", "marking_data", "marking_node") {
    // Subscribe to world state
    marking_list.fill(kInvalidRobotId); // initializes to no valid markers
    danger_score.fill(std::numeric_limits<double>::infinity()); // everyone starts with an infinite danger score
    world_state_sub_ = this->create_subscription<rj_msgs::msg::WorldState>(
        vision_filter::topics::kWorldStateTopic, rclcpp::QoS(1),
        [this](rj_msgs::msg::WorldState::SharedPtr world_state) {  // NOLINT
            last_world_state_ = rj_convert::convert_from_ros(*world_state);
            publish_marking_list();
        });
}

void Marking::service_callback(RequestPtr request, ResponsePtr response) {
    // complete logic
    // bool membership_changed = wants_to_kick_by_id_[request->robot_id] != request->wants_to_kick;

    // wants_to_kick_by_id_[request->robot_id] = request->wants_to_kick;

    // if (membership_changed) {
    //     publish_selected_kicker();
    // }

    // response->success = true;
}

void Marking::publish_marking_list() {
    // Find all eligible targets and update the list
    // Formula, closer to 0 is more priority:
    // Danger value = 0.2 * distance to goal + 0.4 * distance to ball + 0.7 * distance to closest RJ robot
    // Danger value <= 5 gets you onto the eligible target list (subject to change)


    // double min_distance = std::numeric_limits<double>::infinity();
    // uint8_t selected_kicker = kInvalidRobotId;

    // const auto& ball_pos = last_world_state_.ball.position;

    // for (uint8_t i = 0; i < kNumShells; ++i) {
    //     if (wants_to_kick_by_id_[i]) {
    //         const auto& robot = last_world_state_.get_robot(true, i);
    //         double distance = ball_pos.dist_to(robot.pose.position());
    //         if (distance < min_distance) {
    //             min_distance = distance;
    //             selected_kicker = i;
    //         }
    //     }
    // }

    // // Only publish if the selected kicker has changed
    // if (selected_kicker != last_published_kicker_) {
    //     publisher_->publish(rj_msgs::msg::KickerPicker().set__robot_id(selected_kicker));
    //     last_published_kicker_ = selected_kicker;
    // }
}

void Marking::update_danger_scores() {
    // const auto& ball_pos = last_world_state_.ball.position;

    // for (uint8_t i = 0; i < kNumShells; i++) {
    //     const auto& robot = last_world_state_.get_robot(false, i);
    //     double dist_to_ball_ = ball_pos.dist_to(robot.pose.position());
    //     //double dist_to_goal_ = robot.pose.position().dist_to(field_dimensions_.our_goal_loc());
        
    // }
}


}  // namespace strategy
