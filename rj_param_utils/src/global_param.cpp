#include <rj_param_utils/global_param.hpp>

template <typename ParamType>
bool ROS2GlobalParamProvider::UpdateGlobalParameter(    \
        const std::string& full_name, const ParamType& new_value) {
    // TODO: Check if parameter is in node and update. 
    // TODO: Set on parameter event callback.
    return ;
}

template <typename ParamType>
void ROS2GlobalParamProvider::SetGlobalParameter(
        const std::string& full_name, const ParamType& new_value) {
    node_->declare_parameter(full_name);
    parameters_client->set_parameters(rclcpp::Parameter(full_name, new_value));
}

void ROS2GlobalParamProvider::PropagateParameters(rclcpp::Node* node) {
    // TODO: Loop through parameters of given node and update if required.
}


