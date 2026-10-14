#include "rj_control/motion_control_node.hpp"

namespace control {

MotionControlNode::MotionControlNode()
    : rclcpp::Node("control", rclcpp::NodeOptions{}
                                  .automatically_declare_parameters_from_overrides(true)
                                  .allow_undeclared_parameters(true)),
      param_provider_(this, params::kMotionControlParamModule) {}

void MotionControlNode::initialize_controllers() {
    controllers_.reserve(kNumShells);

    for (int i = 0; i < kNumShells; i++) {
        controllers_.emplace_back(i, shared_from_this());
    }
}

}  // namespace control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto control = std::make_shared<control::MotionControlNode>();
    control->initialize_controllers();
    start_global_param_provider(control.get(), kGlobalParamServerNode);
    rclcpp::spin(control);
}