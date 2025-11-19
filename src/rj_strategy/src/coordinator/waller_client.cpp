#include "rj_strategy/coordinator/waller_client.hpp"

namespace strategy {
    
WallerClient::WallerClient(rclcpp::Node::SharedPtr node, uint8_t robot_id) : node_{std::move(node)}, robot_id_{robot_id} {
    client_ = node_->create_client<rj_msgs::srv::Waller>("waller_srv");
    walling_robots_.fill(-1);
}

void WallerClient::join_group(StatusCallback callback) {
    if (am_i_member_ || request_pending_) {
        return;
    }

    if (!client_->wait_for_service(std::chrono::seconds(1))) {
        SPDLOG_ERROR("Waller service not available.");
        if (callback) {
            callback(Result{false});
        }
        return;
    }

    auto request = std::make_shared<rj_msgs::srv::Waller::Request>();
    request->robot_id = robot_id_;
    request->joining = true;

    request_pending_ = true;
    client_->async_send_request(request, [this, callback = std::move(callback)](
        rclcpp::Client<rj_msgs::srv::Waller>::SharedFuture future) {    // 6 NOLINT(performance-unnecessary-value-param) --
                                                                        //  ROS2 async callbacks require value capture.
            request_pending_ = false;
            if (!future.valid() || !future.get()->success) {
                if (callback) {
                    callback(Result{false});
                }
            }

            am_i_member_ = true;

            subscription_ = node_->create_subscription<rj_msgs::msg::Waller>(
                "waller_data", rclcpp::QoS(1).transient_local(),
                [this, callback = std::move(callback)] 
                    (const rj_msgs::msg::Waller::SharedPtr msg) {
                        walling_robots_ = msg->wall_list;
                        num_wallers_ = msg->wall_size;
                        am_i_member_ = std::find(walling_robots_.begin(), walling_robots_.end(), robot_id_) != walling_robots_.end();
                        
                        if (callback) {
                            callback(Result{am_i_member_});
                        }
                    }
            );

        }
    );
}

void WallerClient::leave_group(StatusCallback callback) {
    if (!am_i_member_ && !request_pending_) {
        if (callback) {
            callback(Result{false});
        }
        return;
    }

    auto request = std::make_shared<rj_msgs::srv::Waller::Request>();
    request->robot_id = robot_id_;
    request->joining = false;

    client_->async_send_request(
        request, [this, callback = std::move(callback)](
                     rclcpp::Client<rj_msgs::srv::Waller>::SharedFuture
                         future) {  // 6 NOLINT(performance-unnecessary-value-param) --
                                    //  ROS2 async callbacks require value capture.
            if (!future.valid() || !future.get()->success) {
                if (callback) {
                    callback(Result{false});
                }
                return;
            }

            am_i_member_ = false;

            subscription_.reset();
            walling_robots_.fill(-1);
            num_wallers_ = 0;

            if (callback) {
                callback(Result{false});
            }
        });
}

bool WallerClient::am_i_member() const { return am_i_member_; }

std::optional<rj_geometry::Point> WallerClient::get_walling_point(const WorldState* world_state,
                                        FieldDimensions field_dimensions) const {
    if (!am_i_member_) return std::nullopt;
    
    // Creates Minimum wall radius is slightly greater than  box bounds
    // Dimension accessors should be edited when we figure out how we are doing dimensions realtime
    // from vision
    float box_w{field_dimensions.penalty_long_dist()};
    float box_h{field_dimensions.penalty_short_dist()};
    float line_w{field_dimensions.line_width()};
    double min_wall_rad{(kRobotRadius * 4.0f) + line_w +
                        hypot(static_cast<double>(box_w) / 2, static_cast<double>((box_h)))};

    auto ball_pos = world_state->ball.position;

    auto robot_pos = world_state->get_robot(true, robot_id_).pose.position();
    auto goal_pos = rj_geometry::Point{0, 0};

    // Find ball_direction unit vector
    rj_geometry::Point ball_dir_vector{(ball_pos - goal_pos)};

    ball_dir_vector = ball_dir_vector.normalized();

    // Find target Point
    rj_geometry::Point mid_point{(goal_pos) + (ball_dir_vector * min_wall_rad)};

    auto wall_spacing = kRobotDiameterMultiplier * kRobotDiameter + kBallRadius;

    auto it = std::find(walling_robots_.begin(), walling_robots_.end(), robot_id_);
    auto waller_pos = std::distance(walling_robots_.begin(), it)+1;

    rj_geometry::Point target_point{};
    auto angle = (mid_point - goal_pos).angle();
    auto delta_angle = (wall_spacing * (waller_pos - num_wallers_ / 2. - 0.5)) / min_wall_rad;
    auto target_angle = angle - delta_angle;

    target_point =
        (goal_pos + rj_geometry::Point{1, 0}).normalized(min_wall_rad).rotated(target_angle);

    if (abs(robot_pos.dist_to(goal_pos) - min_wall_rad) < kRobotRadius &&
        robot_pos.dist_to(target_point) > kRobotRadius) {
        uint8_t parent_id =
            [&]() {  // Assigning a value to avoid any undefined behavior; will be changed
                if (target_point.x() < robot_pos.x() && waller_pos > 1 &&
                    waller_pos <= num_wallers_) {
                    return static_cast<uint8_t>(walling_robots_[waller_pos - 2]);
                } else if (target_point.x() >= robot_pos.x() && waller_pos >= 1 &&
                           waller_pos < num_wallers_) {
                    return static_cast<uint8_t>(walling_robots_[waller_pos]);
                } else {
                    return static_cast<uint8_t>(robot_id_);
                }
            }();

        if ((target_point.x() < robot_pos.x() && waller_pos != 1) ||
            (target_point.x() > robot_pos.x() && waller_pos != num_wallers_)) {
            auto parent_point = world_state->get_robot(true, parent_id).pose.position();
            angle = (parent_point - goal_pos).angle();
            delta_angle = wall_spacing / min_wall_rad;
            target_angle =
                angle + delta_angle * (signbit(target_point.x() - robot_pos.x()) ? -1 : 1);

            target_point = (goal_pos + rj_geometry::Point{1, 0})
                               .normalized(min_wall_rad)
                               .rotated(target_angle);
        }
    }

    return target_point;
}


} // namespace strategy
