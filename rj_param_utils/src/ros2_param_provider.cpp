#include <rj_param_utils/ros2_param_provider.h>

namespace params {

ROS2ParamProvider::ROS2ParamProvider(rclcpp::Node* node) {
    DeclareParameters(node);
    InitUpdateParamCallbacks(node);
}

void ROS2ParamProvider::DeclareParameters(rclcpp::Node* node) {
#define DECLARE_PARAMS(type)                                        \
    for (const auto& [param_name, param] : GetParamMap<type>()) {   \
        rcl_interfaces::msg::ParameterDescriptor descriptor;        \
        descriptor.description = param->help();                     \
        node->declare_parameter(param_name, param->default_value(), \
                                descriptor);                        \
    }

    // Declare the parameters
    DECLARE_PARAMS(bool)
    DECLARE_PARAMS(int64_t)
    DECLARE_PARAMS(double)
    DECLARE_PARAMS(std::string)
    DECLARE_PARAMS(std::vector<uint8_t>)
    DECLARE_PARAMS(std::vector<bool>)
    DECLARE_PARAMS(std::vector<int64_t>)
    DECLARE_PARAMS(std::vector<double>)
    DECLARE_PARAMS(std::vector<std::string>)

#undef DECLARE_PARAMS
}

void ROS2ParamProvider::InitUpdateParamCallbacks(rclcpp::Node* node) {
    using rcl_interfaces::msg::SetParametersResult;
    const auto on_update = [this](const std::vector<rclcpp::Parameter>& params)
        -> SetParametersResult {
        for (const rclcpp::Parameter& param : params) {
            switch (param.get_type()) {
                case rclcpp::PARAMETER_BOOL:
                    Update(param.get_name(), param.as_bool());
                    break;
                case rclcpp::PARAMETER_INTEGER:
                    Update(param.get_name(), param.as_int());
                    break;
                case rclcpp::PARAMETER_DOUBLE:
                    Update(param.get_name(), param.as_double());
                    break;
                case rclcpp::PARAMETER_STRING:
                    Update(param.get_name(), param.as_string());
                    break;
                case rclcpp::PARAMETER_BYTE_ARRAY:
                    Update(param.get_name(), param.as_byte_array());
                    break;
                case rclcpp::PARAMETER_BOOL_ARRAY:
                    Update(param.get_name(), param.as_bool_array());
                    break;
                case rclcpp::PARAMETER_INTEGER_ARRAY:
                    Update(param.get_name(), param.as_integer_array());
                    break;
                case rclcpp::PARAMETER_DOUBLE_ARRAY:
                    Update(param.get_name(), param.as_double_array());
                    break;
                case rclcpp::PARAMETER_STRING_ARRAY:
                    Update(param.get_name(), param.as_string_array());
                    break;
                default:
                    break;
            }
        }
        SetParametersResult set_parameters_result;
        set_parameters_result.successful = true;
        return set_parameters_result;
    };
    callback_handle_ = node->add_on_set_parameters_callback(on_update);
}
}  // namespace params
