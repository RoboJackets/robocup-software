#include "rj_strategy/coordinator/seeking_client.hpp"

namespace strategy {

SeekingClient::SeekingClient(rclcpp::Node::SharedPtr node, uint8_t robot_id)
    : node_{std::move(node)}, robot_id_{robot_id}, selected_target_{nullptr} {
    client_ = node_->create_client<rj_msgs::srv::SeekingCoordinator>("seeking_coordinator_srv");
}

void SeekingClient::join_group(StatusCallback callback) {
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
        SPDLOG_ERROR("SeekingCoordinator service not available.");
        if (callback) {
            callback(Result{false});
        }
        return;
    }

    if (am_i_member()) return;

    auto request = std::make_shared<rj_msgs::srv::SeekingCoordinator::Request>();
    request->robot_id = robot_id_;
    request->wants_to_seek = true;

    client_->async_send_request(
        request, [this, callback = std::move(callback)](
                     rclcpp::Client<rj_msgs::srv::SeekingCoordinator>::SharedFuture future) {
            if (!future.valid() || !future.get()->success) {
                if (callback) {
                    callback(Result{false});
                }
                return;
            }
            am_i_member_ = true;
            if (callback) {
                callback(Result{true});
            }
            subscription_ = node_->create_subscription<rj_msgs::msg::SeekingCoordinator>(
                "seeking_coordinator_data", rclcpp::QoS(1).transient_local(),
                [this, callback = std::move(callback)](
                    const rj_msgs::msg::SeekingCoordinator::SharedPtr msg) {
                    if (msg->positions[robot_id_].x == -1 && msg->positions[robot_id_].y == -1) {
                        selected_target_ = nullptr;
                    } else {
                        selected_target_ = std::make_shared<rj_geometry::Point>(
                            msg->positions[robot_id_].x, msg->positions[robot_id_].y);
                    }
                });
        });
}

void SeekingClient::leave_group(StatusCallback callback) {
    if (!am_i_member_) {
        if (callback) {
            callback(Result{false});
        }
        return;
    }

    auto request = std::make_shared<rj_msgs::srv::SeekingCoordinator::Request>();
    request->robot_id = robot_id_;
    request->wants_to_seek = false;

    client_->async_send_request(
        request, [this, callback = std::move(callback)](
                     rclcpp::Client<rj_msgs::srv::SeekingCoordinator>::SharedFuture
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

            if (callback) {
                callback(Result{false});
            }
        });
}

bool SeekingClient::am_i_member() const { return am_i_member_; }

std::shared_ptr<rj_geometry::Point> SeekingClient::selected_target() const {
    return selected_target_;
}

}  // namespace strategy
