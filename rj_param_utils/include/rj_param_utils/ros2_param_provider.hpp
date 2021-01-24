#pragma once

#include <rj_param_utils/param.hpp>

namespace params {
/**
 * An implementation of ParamProvider that integrates with the ROS2 parameter
 * system.
 */
class ROS2ParamProvider : public ::params::ParamProvider {
public:
    explicit ROS2ParamProvider(rclcpp::Node* node, const std::string& module);

protected:
    rcl_interfaces::msg::SetParametersResult UpdateParameters(const std::vector<rclcpp::Parameter>& params);

private:
    /**
     * @brief Calls node->declare_parameters on all the registered parameters.
     * @param node
     */
    void DeclareParameters(rclcpp::Node* node);

    /**
     * @brief Initializes the callback called by the ROS2 node ROS2 paramter
     * updates. The callback will update the registered parameters in
     * ParamRegistry.
     * @param node
     */
    void InitUpdateParamCallbacks(rclcpp::Node* node);

    /**
     * @brief Converts the full_name of a parameter from using double colons
     * (::) as the namespace separator to using periods (.).
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

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        callback_handle_;
};
}  // namespace params
