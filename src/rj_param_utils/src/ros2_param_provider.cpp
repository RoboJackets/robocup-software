#include <spdlog/spdlog.h>

#include <rj_param_utils/ros2_param_provider.hpp>

namespace params {

rcl_interfaces::msg::SetParametersResult BaseROS2ParamProvider::UpdateParameters(
    const std::vector<rclcpp::Parameter>& params) {
    using rcl_interfaces::msg::SetParametersResult;

    bool all_succeeded = true;
    for (const rclcpp::Parameter& param : params) {
        std::string full_name = ConvertFullNameFromROS2(param.get_name());
        bool success = false;
        switch (param.get_type()) {
            case rclcpp::PARAMETER_BOOL:
                success = TryUpdate(full_name, param.as_bool());
                break;
            case rclcpp::PARAMETER_INTEGER:
                success = TryUpdate(full_name, param.as_int());
                break;
            case rclcpp::PARAMETER_DOUBLE:
                success = TryUpdate(full_name, param.as_double());
                break;
            case rclcpp::PARAMETER_STRING:
                success = TryUpdate(full_name, param.as_string());
                break;
            case rclcpp::PARAMETER_BYTE_ARRAY:
                success = TryUpdate(full_name, param.as_byte_array());
                break;
            case rclcpp::PARAMETER_BOOL_ARRAY:
                success = TryUpdate(full_name, param.as_bool_array());
                break;
            case rclcpp::PARAMETER_INTEGER_ARRAY:
                success = TryUpdate(full_name, param.as_integer_array());
                break;
            case rclcpp::PARAMETER_DOUBLE_ARRAY:
                success = TryUpdate(full_name, param.as_double_array());
                break;
            case rclcpp::PARAMETER_STRING_ARRAY:
                success = TryUpdate(full_name, param.as_string_array());
                break;
            default:
                break;
        }

        // Failure could happen in two main cases:
        //  - The parameter does not exist
        //  - The parameter's type as declared does not match the parameter's current type
        if (!success) {
            // NOLINTNEXTLINE(bugprone-lambda-function-name)
            SPDLOG_WARN("Failed to set parameter {}", param.get_name());
            all_succeeded = false;
        }
    }

    SetParametersResult set_parameters_result;
    set_parameters_result.successful = all_succeeded;
    return set_parameters_result;
}

std::string BaseROS2ParamProvider::ConvertFullNameToROS2(const std::string& full_name) {
    std::string ros2_name;

    ros2_name.reserve(full_name.size());
    for (size_t char_idx = 0; char_idx < full_name.size() - 1; char_idx++) {
        if (full_name[char_idx] == ':' && full_name[char_idx + 1] == ':') {
            ros2_name.push_back('.');
            char_idx++;
        } else {
            ros2_name.push_back(full_name[char_idx]);
        }
    }
    ros2_name.push_back(full_name.back());

    return ros2_name;
}

std::string BaseROS2ParamProvider::ConvertFullNameFromROS2(const std::string& ros2_name) {
    std::string full_name;

    full_name.reserve(ros2_name.size());
    for (char c : ros2_name) {
        if (c == '.') {
            full_name.append("::");
        } else {
            full_name.push_back(c);
        }
    }
    return full_name;
}

}  // namespace params
