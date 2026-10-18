#include "rj_control/motion_control_node.hpp"

#include <rj_common/ros_params.hpp>

namespace control {

MotionControlNode::MotionControlNode()
    : rclcpp::Node("control") {
}

void MotionControlNode::initialize_controllers() {
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

    for (size_t i = 0; i < kNumShells; i++) {
        controllers_.push_back(
            std::make_unique<MotionControl>(static_cast<int>(i), this, limits));
    }
}

}  // namespace control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto control = std::make_shared<control::MotionControlNode>();
    control->initialize_controllers();
    rclcpp::spin(control);
}