#include "rj_control/motion_control_node.hpp"

#include <rj_utils/ros_params.hpp>

namespace control {

MotionControlNode::MotionControlNode() : rclcpp::Node("control") {
    using rj_utils::declare_required_param;
    // Shared by every robot's controller, so declared once here.
    const MotionControlLimits limits{
        declare_required_param<double>(*this, "max_velocity", "Max commanded speed (m/s)"),
        declare_required_param<double>(*this, "max_angular_velocity",
                                       "Max commanded angular speed (rad/s)"),
        declare_required_param<int>(*this, "translation_windup",
                                    "Translation PID integral windup limit (0 = unlimited)"),
        declare_required_param<int>(*this, "rotation_windup",
                                    "Rotation PID integral windup limit (0 = unlimited)"),
    };

    controllers_.reserve(kNumShells);

    auto drawing_publisher = create_publisher<rj_drawing_msgs::msg::DebugDraw>(
        viz::topics::kDebugDrawTopic, rclcpp::QoS(10));
    for (int i = 0; i < kNumShells; i++) {
        controllers_.push_back(std::make_unique<MotionControl>(i, this, limits));
    }
}

}  // namespace control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto control = std::make_shared<control::MotionControlNode>();
    rclcpp::spin(control);
}