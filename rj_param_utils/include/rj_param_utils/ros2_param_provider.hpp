#pragma once

#include <rj_param_utils/param.hpp>

namespace params {
/**
 * The base for parameter providers using ROS parameters.
 *
 * Provides useful helper methods.
 */
class BaseROS2ParamProvider : public ::params::ParamProvider {
protected:
    BaseROS2ParamProvider(const std::string& module) : ParamProvider{module} {}

    /**
     * Update the parameters provided by this object with a key/value list.
     * @param params the parameters to set. All parameters not included in this vector will be left
     * unchanged.
     * @return a success/fail result. If any parameter fails the whole operation is assumed to have
     * failed.
     */
    rcl_interfaces::msg::SetParametersResult UpdateParameters(
        const std::vector<rclcpp::Parameter>& params);

    /**
     * @brief Converts the full_name of a parameter from using double colons (::) as the namespace
     * separator to using periods (.).
     * @param full_name The full_name to convert.
     * @return The full name of the parameter in ROS2 parameter convention.
     */
    static std::string ConvertFullNameToROS2(const std::string& full_name);

    /**
     * @brief Converts the full name of a parameter from using periods (.) as the namespace separator to using double colons (::). Inverts @ref ConvertFullNameToROS2.
     * @param ros2_name The ros2 name to convert.
     * @return The full name of the parameter in ROS2 parameter convention.
     */
    static std::string ConvertFullNameFromROS2(const std::string& ros2_name);
};

}  // namespace params
