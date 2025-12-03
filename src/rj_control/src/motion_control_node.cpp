#include "rj_control/motion_control_node.hpp"

namespace control {

MotionControlNode::MotionControlNode()
    : rclcpp::Node("control", rclcpp::NodeOptions{}
                                  .automatically_declare_parameters_from_overrides(true)
                                  .allow_undeclared_parameters(true)),
      param_provider_(this, params::kMotionControlParamModule) {
    controllers_.reserve(kNumShells);

    auto drawing_publisher = create_publisher<rj_drawing_msgs::msg::DebugDraw>(
        viz::topics::kDebugDrawTopic, rclcpp::QoS(10));
    for (int i = 0; i < kNumShells; i++) {
        try {
            controllers_.emplace_back(i, this);
        } catch (const std::exception& e) {
            SPDLOG_ERROR("Failed to construct MotionControl for id {}: {}", i, e.what());
        } catch (...) {
            SPDLOG_ERROR("Failed to construct MotionControl for id {}: unknown error", i);
        }
    }
}

}  // namespace control

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    // Start the global param provider receiver thread first so parameters
    // from the global param server are available before we construct
    // MotionControlNode and its per-robot controllers.
    start_global_param_provider("processor", kGlobalParamServerNode);

    auto control = std::make_shared<control::MotionControlNode>();
    rclcpp::spin(control);
}