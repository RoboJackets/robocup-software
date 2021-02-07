#include <rj_param_utils/param.hpp>
#include <rj_param_utils/global_param.hpp>
#include <rj_constants/topic_names.hpp>

params::ROS2GlobalParamProvider::ROS2GlobalParamProvider(rclcpp::Node* node, 
        const std::string& global_node) : params::ROS2ParamProvider{node, global_node} {
    node_ = node;
    params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node);

    using rcl_interfaces::msg::SetParametersResult;

    auto on_param_event_callback = 
        [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
           return UpdateParameters(event->changed_parameters); 
        };

    params_client_sub_ = params_client_->on_parameter_event(on_param_event_callback);
}


