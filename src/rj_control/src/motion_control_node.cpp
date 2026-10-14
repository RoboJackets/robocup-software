#include "rj_control/motion_control_node.hpp"

#include <rj_utils/ros_params.hpp>

namespace control {

MotionControlNode::MotionControlNode() : rclcpp::Node("control") {
    using rj_utils::declare_required_param;
    // Shared by every robot's controller, so declared once here.
    limits_ = MotionControlLimits{
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
        controllers_.push_back(std::make_unique<MotionControl>(i, this, limits_));
    }

    param_callback_handle_ =
        add_on_set_parameters_callback([this](const std::vector<rclcpp::Parameter>& params) {
            rcl_interfaces::msg::SetParametersResult result;
            result.successful = true;

            for (const auto& param : params) {
                const auto& name = param.get_name();
                if ((name == "max_velocity" || name == "max_angular_velocity") &&
                    param.as_double() < 0) {
                    result.successful = false;
                    result.reason = name + " must be nonnegative";
                    return result;
                }
                if ((name == "translation_windup" || name == "rotation_windup") &&
                    param.as_int() < 0) {
                    result.successful = false;
                    result.reason = name + " must be nonnegative";
                    return result;
                }
            }

            for (const auto& param : params) {
                const auto& name = param.get_name();
                if (name == "max_velocity") {
                    limits_.max_velocity = param.as_double();
                } else if (name == "max_angular_velocity") {
                    limits_.max_angular_velocity = param.as_double();
                } else if (name == "translation_windup") {
                    limits_.translation_windup = param.as_int();
                } else if (name == "rotation_windup") {
                    limits_.rotation_windup = param.as_int();
                }
            }

            for (const auto& controller : controllers_) {
                controller->limits_ = limits_;
                controller->update_gains(params);
            }

            return result;
        });
}

}  // namespace control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto control = std::make_shared<control::MotionControlNode>();
    rclcpp::spin(control);
}