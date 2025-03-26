#pragma once

#include "kicker_picker_client.hpp"

#include <utility>

#include <rclcpp/rclcpp.hpp>
#include <spdlog/spdlog.h>

#include <rj_msgs/msg/kicker_picker.hpp>
#include <rj_msgs/srv/kicker_picker.hpp>

#include "kicker_picker.hpp"

namespace strategy {

/**
 * @brief Client for interacting with the KickerPicker coordinator.
 *
 * Manages membership in the kicker group and tracks the currently selected kicker.
 * All callbacks are executed in the node's callback group, so no mutex is needed
 * for thread safety in single-threaded execution.
 */

KickerPickerClient::KickerPickerClient(rclcpp::Node::SharedPtr node, uint8_t robot_id)
    : node_{std::move(node)}, robot_id_{robot_id}, selected_kicker_{KickerPicker::kInvalidRobotId} {
    client_ = node_->create_client<rj_msgs::srv::KickerPicker>("kicker_picker_srv");
}

void KickerPickerClient::join_group(StatusCallback callback) {
    if (am_i_member_) {
        if (callback) {
            callback(MembershipStatus{true});
        }
        return;
    }

    if (!client_->wait_for_service(std::chrono::seconds(1))) {
        SPDLOG_ERROR("KickerPicker service not available.");
        if (callback) {
            callback(MembershipStatus{false});
        }
        return;
    }

    auto request = std::make_shared<rj_msgs::srv::KickerPicker::Request>();
    request->robot_id = robot_id_;
    request->wants_to_kick = true;

    client_->async_send_request(
        request, [this, callback](rclcpp::Client<rj_msgs::srv::KickerPicker>::SharedFuture
                                      future) {  // NOLINT(performance-unnecessary-value-param) --
                                                 // ROS2 async callbacks require value capture.
            if (!future.valid() || !future.get()->success) {
                if (callback) {
                    callback(MembershipStatus{false});
                }
                return;
            }

            am_i_member_ = true;

            // Create subscription to track selected kicker.
            subscription_ = node_->create_subscription<rj_msgs::msg::KickerPicker>(
                "kicker_picker_data", rclcpp::QoS(1).best_effort().transient_local(),
                [this](const rj_msgs::msg::KickerPicker::SharedPtr& msg) {
                    selected_kicker_ = msg->robot_id;
                });

            if (callback) {
                callback(MembershipStatus{true});
            }
        });
}

void KickerPickerClient::leave_group(StatusCallback callback) {
    if (!am_i_member_) {
        if (callback) {
            callback(MembershipStatus{false});
        }
        return;
    }

    auto request = std::make_shared<rj_msgs::srv::KickerPicker::Request>();
    request->robot_id = robot_id_;
    request->wants_to_kick = false;

    client_->async_send_request(
        request, [this, callback](rclcpp::Client<rj_msgs::srv::KickerPicker>::SharedFuture
                                      future) {  // NOLINT(performance-unnecessary-value-param) --
                                                 // ROS2 async callbacks require value capture.
            if (!future.valid() || !future.get()->success) {
                if (callback) {
                    callback(MembershipStatus{am_i_member_});
                }
                return;
            }

            am_i_member_ = false;

            // Resetting the shared ptr releases our pointer to the subscription.
            // ROS only keeps a weak_ptr, so this will deallocate the subscription.
            // The callback will no longer be called.
            subscription_.reset();
            selected_kicker_ = KickerPicker::kInvalidRobotId;

            if (callback) {
                callback(MembershipStatus{false});
            }
        });
}

bool KickerPickerClient::am_i_member() const { return am_i_member_; }

uint8_t KickerPickerClient::selected_kicker() const { return selected_kicker_; }

bool KickerPickerClient::is_selected() const { return selected_kicker_ == robot_id_; }

}  // namespace strategy


// class Position {

//     void state_to_task() {

//         //  I want to kicker-pick
//         // option 1:
//         client.join_group() // blocking
//         if (client.am_i_member()) {
//             // advance state
//         }

//         // option 2:
//         client.join_group([](bool am_i_member){
//             if (am_i_member) {
//                 // am kicking member
//                 state_ = BLAH;
//             }
//         });
//         // advance state
//         // state WAITING:
//         // ??? hang out
//         // state AM_KICKING_MEMBER:
//         if (client.is_kicker()) {
//             // kick
//         } else {
//             // hangout
//         }

//         // option 3:
//         client.join_group();
//         // advance state
//         // state WAITING:
//         if (client.am_i_member()) {
//             // advance state
//             // am kicking member
//         }

//         // option 4:
//         client.join_group([](std::optional<int> wall_position){
//             if (wall_position.has_value()) {
//                 // wall in position;
//             } else {
//                 // sub was never made; do something else
//             }
//         });
//         // state WAITING



//         // inside state_to_task() function

//         if current_state_ == SEEKING:
//             if client.is_kicker(): 
//                 state = STEALING;
//             else if !client.am_i_member() && i_think_i_want_to_steal():
//                 client.join_group()... ??

//         // next_state():
        
        

//         client.join_group();
//     }

// }