#include "coach_sub.hpp"

namespace ros2_temp {

CoachSub::CoachSub(Context* context, rclcpp::Executor* executor) : context_(context) {
    node_ = std::make_shared<rclcpp::Node>("_coach_receiver");
    executor->add_node(node_, true);

    auto keep_latest = rclcpp::QoS(1);

    positions_sub_ = node_->create_subscription<rj_msgs::msg::PositionAssignment>(
        "strategy/positions", keep_latest, [this](rj_msgs::msg::PositionAssignment::UniquePtr msg) {
            rj_convert::convert_from_ros(msg->client_positions, &context_->robot_positions);
        });
}

}  // namespace ros2_temp
