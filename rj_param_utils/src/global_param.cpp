#include <rj_param_utils/global_param.hpp>

ROS2GlobalParameterProvider::ROS2GlobalParamProvider(rclcpp::Node* node, 
        const rclcpp::Node* global_node) : node_{node}, global_node_{global_node} {
    params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node);

    params_client_sub_ = params_client_->on_parameter_event(UpdateGlobalParams);
}

template <typename ParamType>
void ROS2GlobalParamProvider::UpdateGlobalParams() {
    // TODO: Update parameters in the global params node.
}




