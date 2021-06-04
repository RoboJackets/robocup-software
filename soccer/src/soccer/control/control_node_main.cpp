#include <rj_utils/logging.hpp>

#include "global_params.hpp"
#include "motion_control_node.hpp"

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto control = std::make_shared<control::MotionControlNode>();
    start_global_param_provider(control.get(), kGlobalParamServerNode);
    rclcpp::spin(control);
}
