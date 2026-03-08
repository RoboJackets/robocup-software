#include "rj_testing/rotate_test_node.hpp"

namespace rj_testing {

RotateTestNode::RotateTestNode()
    : rclcpp::Node("rotate_test"),
      robot_id_(rj_utils::parse_id_from_namespace(get_namespace()))
{
    action_pub_ = create_publisher<action::Action::Msg>(
        "action", rclcpp::QoS(1).transient_local()
    );
    action_complete_sub_ = create_subscription<std_msgs::msg::Bool>(
        "action/complete", rclcpp::QoS(1).transient_local(),
        //NOLINTNEXTLINE(performance-unnecessary-value-param)
        [this](const std_msgs::msg::Bool::SharedPtr msg) {
            if (msg->data) {
                heading_ *= -1;
                auto rotate = action::Action::create_rotate_to_heading(heading_, 0.05);
                action_pub_->publish(rj_convert::convert_to_ros(rotate));
            }
        }
    );
}

} // namespace rj_testing

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto node = std::make_shared<rj_testing::RotateTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}