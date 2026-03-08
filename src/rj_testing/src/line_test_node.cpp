#include "rj_testing/line_test_node.hpp"

namespace rj_testing {

LineTestNode::LineTestNode()
    : rclcpp::Node("line_test"),
      robot_id_(rj_utils::parse_id_from_namespace(get_namespace()))
{
    action_pub_ = create_publisher<action::Action::Msg>(
        "action", rclcpp::QoS(1).transient_local()
    );
    field_dimensions_sub_ = create_subscription<FieldDimensions::Msg>(
        "/config/field_dimensions", rclcpp::QoS(1).transient_local(),
        //NOLINTNEXTLINE(performance-unnecessary-value-param)
        [this](const FieldDimensions::Msg::SharedPtr msg) {
            field_dimensions_ = rj_convert::convert_from_ros(*msg);
            rj_geometry::Point target = calculate_desired_target();
            auto go_to_point = action::Action::create_go_to_point(target, false);
            action_pub_->publish(rj_convert::convert_to_ros(go_to_point));
        }
    );
    action_complete_sub_ = create_subscription<std_msgs::msg::Bool>(
        "action/complete", rclcpp::QoS(1).transient_local(),
        //NOLINTNEXTLINE(performance-unnecessary-value-param)
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (msg->data) {
                if (direction_ == Direction::LEFT) {
                    direction_ = Direction::RIGHT;
                } else {
                    direction_ = Direction::LEFT;
                }
                rj_geometry::Point target = calculate_desired_target();
                auto go_to_point = action::Action::create_go_to_point(target, false);
                action_pub_->publish(rj_convert::convert_to_ros(go_to_point));
            }
        }
    );
}

rj_geometry::Point LineTestNode::calculate_desired_target() {
    rj_geometry::Point target;
    if (direction_ == Direction::LEFT) {
        target = {-1.0, 0.0};
    } else {
        target = {1.0, 0.0};
    }
    return target;
}

} // namespace rj_testing

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto node = std::make_shared<rj_testing::LineTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}