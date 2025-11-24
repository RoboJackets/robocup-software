#include "rj_strategy/coordinator/seeker_client.hpp"

namespace strategy {

SeekerClient::SeekerClient(rclcpp::Node::SharedPtr node, uint8_t robot_id)
    : node_{std::move(node)}, robot_id_{robot_id}, selected_target_{-1,-1} {
    client_ = node_->create_client<rj_msgs::srv::SeekerCoordinator>("seeker_coordinator_srv");
}

void SeekerClient::poll_for_target(StatusCallback callback) {
    if (!client_->wait_for_service(std::chrono::seconds(1))) {
        SPDLOG_ERROR("SeekerCoordinator service not available.");
        if (callback) {
            callback(Result{false});
        }
        return;
    }

    auto request = std::make_shared<rj_msgs::srv::SeekerCoordinator::Request>();
    request->robot_id = robot_id_;
    request->wants_to_seek = true;

    client_->async_send_request(request, [this, callback = std::move(callback)](
        rclcpp::Client<rj_msgs::srv::SeekerCoordinator>::SharedFuture future) {
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
            subscription_ = node_->create_subscription<rj_msgs::msg::SeekerCoordinator>(
            "seeker_coordinator_data", rclcpp::QoS(1).transient_local(),
            [this, callback=std::move(callback)](const rj_msgs::msg::SeekerCoordinator::SharedPtr msg) {
                selected_target_ = rj_geometry::Point{msg->positions[robot_id_].x, msg->positions[robot_id_].y};
            });
        });

}

void SeekerClient::leave_group(StatusCallback callback) {
    if (!am_i_member_) {
        if (callback) {
            callback(Result{false});
        }
        return;
    }

    auto request = std::make_shared<rj_msgs::srv::SeekerCoordinator::Request>();
    request->robot_id = robot_id_;
    request->wants_to_seek = false;

    client_->async_send_request(
        request, [this, callback = std::move(callback)](
                     rclcpp::Client<rj_msgs::srv::SeekerCoordinator>::SharedFuture
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
            selected_target_ = SeekerCoordinator::invalidPoint();

            if (callback) {
                callback(Result{false});
            }
        });
}

bool SeekerClient::am_i_member() const { return am_i_member_; }

rj_geometry::Point SeekerClient::selected_target() const { return selected_target_; }


}  // namespace strategy
