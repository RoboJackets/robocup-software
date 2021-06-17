#include <rclcpp/rclcpp.hpp>

#include <rj_param_utils/ros2_local_param_provider.hpp>

#include "global_params.hpp"

int main(int argc, const char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>(
        kGlobalParamServerNode, rclcpp::NodeOptions{}
                                    .allow_undeclared_parameters(true)
                                    .automatically_declare_parameters_from_overrides(true));
    auto provider = params::LocalROS2ParamProvider(node.get(), kGlobalParamModule);
    rclcpp::spin(node);
    rclcpp::shutdown();
    global_params_server_dummy_function();
    return 0;
}
