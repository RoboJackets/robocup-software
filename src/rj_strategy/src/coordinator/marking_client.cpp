#include "rj_strategy/coordinator/marking_client.hpp"

namespace strategy {

/**
 * @brief Client for interacting with the Marking coordinator.
 *
 * Manages membership in the kicker group and tracks the currently selected kicker.
 */

MarkingClient::MarkingClient(rclcpp::Node::SharedPtr node, uint8_t robot_id)
    : node_{std::move(node)},
      robot_id_{robot_id},
      selected_robot_marking_id_{MarkingClient::kInvalidRobotId} {
    client_ = node_->create_client<rj_msgs::srv::Marking>("marking_srv");
}

void MarkingClient::join_group(StatusCallback callback) {
    if (am_i_member_) {
        return;
    }

    if (!client_->wait_for_service(std::chrono::seconds(1))) {
        SPDLOG_ERROR("Marking service not available.");
        if (callback) {
            callback(Result{false});
        }
        return;
    }

    auto request = std::make_shared<rj_msgs::srv::Marking::Request>();
    request->robot_id = robot_id_;
    request->join = true;

    client_->async_send_request(
        request, [this, callback = std::move(callback)](
                     rclcpp::Client<rj_msgs::srv::Marking>::SharedFuture
                         future) {  // 6 NOLINT(performance-unnecessary-value-param) --
                                    //  ROS2 async callbacks require value capture.
            if (!future.valid() || !future.get()->success) {
                if (callback) {
                    callback(Result{false});
                }
                return;
            }

            am_i_member_ = true;

            // Create subscription to track selected kicker.
            subscription_ = node_->create_subscription<rj_msgs::msg::Marking>(
                "marking_data", rclcpp::QoS(1).transient_local(),
                [this](const rj_msgs::msg::Marking::SharedPtr msg) {  // callback=std::move(callback)
                    selected_robot_marking_id_ = msg->mark_robot_ids[robot_id_];
                    am_i_marking_ = (selected_robot_marking_id_ != kInvalidRobotId);
                    // callback(Result{true, selected_robot_marking_id_});
                });

            callback(Result{true});

        });
}

void MarkingClient::leave_group(StatusCallback callback) {
    if (!am_i_member_) {
        if (callback) {
            callback(Result{false});
        }
        return;
    }

    auto request = std::make_shared<rj_msgs::srv::Marking::Request>();
    request->robot_id = robot_id_;
    request->join = false;

    client_->async_send_request(
        request, [this, callback = std::move(callback)](
                     rclcpp::Client<rj_msgs::srv::Marking>::SharedFuture
                         future) {  // 6 NOLINT(performance-unnecessary-value-param) --
                                    //  ROS2 async callbacks require value capture.
            if (!future.valid() || !future.get()->success) {
                if (callback) {
                    callback(Result{false});
                }
                return;
            }

            am_i_member_ = false;

            // Resetting the shared ptr releases our pointer to the
            // subscription. ROS only keeps a weak_ptr, so this will
            // deallocate the subscription. The callback will no longer be
            // called.
            subscription_.reset();
            am_i_marking_ = false;
            selected_robot_marking_id_ = kInvalidRobotId;

            if (callback) {
                callback(Result{true, selected_robot_marking_id_});
            }
        });
}

bool MarkingClient::am_i_member() const { return am_i_member_; }

uint8_t MarkingClient::who_am_i_marking() const { return selected_robot_marking_id_; }

bool MarkingClient::am_i_marking() const { return am_i_marking_; }

}  // namespace strategy
