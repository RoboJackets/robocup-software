#include "rj_param_utils/ros2_local_param_provider.hpp"

namespace params {

LocalROS2ParamProvider::LocalROS2ParamProvider(rclcpp::Node* node, const std::string& module)
    : BaseROS2ParamProvider{module} {
    DeclareParameters(node);
    InitUpdateParamCallbacks(node);
}

#define DECLARE_AND_UPDATE_PARAMS(type)                                                   \
    for (const auto& [param_name, param] : GetParamMap<type>()) {                         \
        rcl_interfaces::msg::ParameterDescriptor descriptor;                              \
        descriptor.description = param->help();                                           \
        const std::string& ros2_param_name = ConvertFullNameToROS2(param_name);           \
        const type& val =                                                                 \
            node->declare_parameter(ros2_param_name, param->default_value(), descriptor); \
        Update(param_name, val);                                                          \
    }

void LocalROS2ParamProvider::DeclareParameters(rclcpp::Node* node) {
    // Declare the parameters
    DECLARE_AND_UPDATE_PARAMS(bool)
    DECLARE_AND_UPDATE_PARAMS(int64_t)
    DECLARE_AND_UPDATE_PARAMS(double)
    DECLARE_AND_UPDATE_PARAMS(std::string)
    DECLARE_AND_UPDATE_PARAMS(std::vector<uint8_t>)
    DECLARE_AND_UPDATE_PARAMS(std::vector<bool>)
    DECLARE_AND_UPDATE_PARAMS(std::vector<int64_t>)
    DECLARE_AND_UPDATE_PARAMS(std::vector<double>)
    DECLARE_AND_UPDATE_PARAMS(std::vector<std::string>)
}
#undef DECLARE_AND_UPDATE_PARAMS

void LocalROS2ParamProvider::InitUpdateParamCallbacks(rclcpp::Node* node) {
    using rcl_interfaces::msg::SetParametersResult;
    const auto on_update = [this](const std::vector<rclcpp::Parameter>& params) {
        return UpdateParameters(params);
    };
    callback_handle_ = node->add_on_set_parameters_callback(on_update);
}

}  // namespace params
