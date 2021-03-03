#include <rj_param_utils/param.hpp>
#include <rj_param_utils/global_param.hpp>

params::ROS2GlobalParamProvider::ROS2GlobalParamProvider(rclcpp::Node* node, 
        const std::string& global_node) : params::ROS2ParamProvider{node, global_node}, global_node_(global_node) {
    node_ = node;
    params_client_ = std::make_shared<rclcpp::AsyncParametersClient>(node);

    using rcl_interfaces::msg::SetParametersResult;

    auto on_param_event_callback = 
        [this](const rcl_interfaces::msg::ParameterEvent::SharedPtr event) {
           return UpdateParameters(this->ConvertToParam(event->changed_parameters));
        };

    params_client_sub_ = params_client_->on_parameter_event(on_param_event_callback);
}

template<typename ParamType>
std::vector<rclcpp::Parameter> params::ROS2GlobalParamProvider::ConvertToParam(const std::vector<rcl_interfaces::msg::Parameter_<ParamType>>& param_msgs) {
    std::vector<rclcpp::Parameter> parameters;
    parameters.reserve(param_msgs.size());
    for(const auto param_msg : param_msgs) {
        rclcpp::Parameter new_param = rclcpp::Parameter(param_msg.name, param_msg.value);
        parameters.emplace_back(new_param);

    }

    return parameters;
}
