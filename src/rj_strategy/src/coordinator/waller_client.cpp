#include "rj_strategy/coordinator/waller_client.hpp"

namespace strategy {

WallerClient::WallerClient(rclcpp::Node::SharedPtr node, uint8_t robot_id)
    : node_{std::move(node)}, robot_id_{robot_id} {
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
    client_->async_send_request(
        request, [this, callback = std::move(callback)](
                     rclcpp::Client<rj_msgs::srv::Waller>::SharedFuture
                         future) {  // 6 NOLINT(performance-unnecessary-value-param) --
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
                [this, callback = std::move(callback)](const rj_msgs::msg::Waller::SharedPtr msg) {
                    walling_robots_ = msg->wall_list;
                    num_wallers_ = msg->wall_size;
                    am_i_member_ = std::find(walling_robots_.begin(), walling_robots_.end(),
                                             robot_id_) != walling_robots_.end();

                    if (callback) {
                        callback(Result{am_i_member_});
                    }
                });

        });
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
                callback(Result{true});
            }
        });
}

bool WallerClient::am_i_member() const { return am_i_member_; }

std::optional<rj_geometry::Point> WallerClient::get_walling_point(
    const WorldState* world_state, FieldDimensions field_dimensions) const {
    if (!am_i_member_) return std::nullopt;

    auto waller_geometry = calculate_wall_geometry(world_state, field_dimensions);
    auto waller_pos =
        std::distance(walling_robots_.begin(),
                      std::find(walling_robots_.begin(), walling_robots_.end(), robot_id_));

    // Target point where the waller should end up
    auto target_point = get_target_position(waller_geometry, waller_pos);

    // If we can, follow our parent instead of going directly to the target
    if (auto parent_id = get_parent_id(waller_geometry, target_point, waller_pos)) {
        auto parent_point = world_state->get_robot(true, *parent_id).pose.position();
        return get_target_position_with_parent(waller_geometry, target_point, parent_point);
    }

    return target_point;
}

WallerClient::WallerGeometry WallerClient::calculate_wall_geometry(
    const WorldState* world_state, FieldDimensions field_dimensions) const {
    WallerGeometry waller_geometry{};

    // Creates Minimum wall radius is slightly greater than  box bounds
    // Dimension accessors should be edited when we figure out how we are doing dimensions realtime
    // from vision
    float box_w{field_dimensions.penalty_long_dist()};
    float box_h{field_dimensions.penalty_short_dist()};
    float line_w{field_dimensions.line_width()};
    waller_geometry.min_wall_radius =
        (kRobotRadius * 4.0f) + line_w +
        hypot(static_cast<double>(box_w) / 2, static_cast<double>((box_h)));

    waller_geometry.robot_pos = world_state->get_robot(true, robot_id_).pose.position();
    waller_geometry.goal_pos = field_dimensions.our_goal_loc();
    waller_geometry.ball_pos = world_state->ball.position;
    waller_geometry.wall_spacing = kRobotDiameterMultiplier * kRobotDiameter + kBallRadius;

    return waller_geometry;
}

rj_geometry::Point WallerClient::get_target_position(WallerGeometry& waller_geometry,
                                                     long waller_pos) const {
    // Find target point of this robot
    rj_geometry::Point ball_dir_vector =
        rj_geometry::Point(waller_geometry.ball_pos - waller_geometry.goal_pos).normalized();

    // This serves as the center of the wall arc
    rj_geometry::Point mid_point{(waller_geometry.goal_pos) +
                                 (ball_dir_vector * waller_geometry.min_wall_radius)};
    auto angle = (mid_point - waller_geometry.goal_pos).angle();

    // Wallers are distributed evenly across the wall arc based on their position in the wall list
    auto delta_angle = (waller_geometry.wall_spacing * (waller_pos - num_wallers_ / 2. - 0.5)) /
                       waller_geometry.min_wall_radius;
    auto target_angle = angle - delta_angle;

    // Calculate the target point using polar coordinates with wall radius and target angle
    return rj_geometry::Point(1, 0)
        .normalized(waller_geometry.min_wall_radius)
        .rotated(target_angle);
}

std::optional<uint8_t> WallerClient::get_parent_id(WallerGeometry& waller_geometry,
                                                   rj_geometry::Point target_point,
                                                   long waller_pos) const {
    // Finds the parent point along the wall
    auto distance_from_arc = abs(waller_geometry.robot_pos.dist_to(waller_geometry.goal_pos) -
                                 waller_geometry.min_wall_radius);
    auto distance_from_target = waller_geometry.robot_pos.dist_to(target_point);

    // We are not along the wall arco or we are close to our target, do not follow the parent
    if (distance_from_arc >= kRobotRadius || distance_from_target <= kRobotRadius)
        return std::nullopt;

    // We need to move to the left so our parent is the robot to the left of us
    if (target_point.x() < waller_geometry.robot_pos.x() && waller_pos > 0)
        return static_cast<uint8_t>(walling_robots_[waller_pos - 1]);
    // We need to move to the right so our parent is the robot to the right of us
    else if (target_point.x() >= waller_geometry.robot_pos.x() && waller_pos < num_wallers_ - 1)
        return static_cast<uint8_t>(walling_robots_[waller_pos + 1]);

    // We are the first robot in the arc, we should not follow anyone
    return std::nullopt;
}

rj_geometry::Point WallerClient::get_target_position_with_parent(
    WallerGeometry& waller_geometry, rj_geometry::Point target_point,
    rj_geometry::Point parent_point) const {
    // Find target point of this robot by following some parent robot
    auto angle = (parent_point - waller_geometry.goal_pos).angle();

    // Go to {wall_spacing} behind the parent
    // Uses signbit() to determine whether the parent is to the left or to the right
    auto delta_angle = waller_geometry.wall_spacing / waller_geometry.min_wall_radius;
    auto target_angle =
        angle + delta_angle * (signbit(target_point.x() - waller_geometry.robot_pos.x()) ? -1 : 1);

    // Calculate the target position using polar coordinates
    return rj_geometry::Point(1, 0)
        .normalized(waller_geometry.min_wall_radius)
        .rotated(target_angle);
}

}  // namespace strategy
