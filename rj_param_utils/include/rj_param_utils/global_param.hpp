/** @file */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rj_param_utils/param.hpp>
#include <rj_param_utils/ros2_param_provider.hpp>

/**
 * @brief Create node to store global parameters.
 */
auto global_param_node = rclcpp::Node::make_shared("global_parameter_node");

// TODO: Get values of constants from physics and update
// global_param_node->declare_parameter("");



namespace params {
/**
 * Implementation of a global parameter server.
 */

class ROS2GlobalParamProvider : public ::params::ROS2ParamProvider {
public:
    explicit ROS2GlobalParamProvider(rclcpp::Node* node, const std::string& module) : node_(node) {}

    /**
     * @brief Updates the global parameter with the passed in full_name and
     * matching type with new_value.
     * returing true if the parameter was found.
     * @tparam ParamType Type of the parameter.
     * @param full_name Name of the parameter to update.
     * @return Whether the parameter was updated successfully.
     */
    template <typename ParamType>
    bool UpdateGlobalParameter(const std::string& full_name, const ParamType& new_value);  

    /**
     * @brief Sets a new global parameter with the passed in full_name and 
     * with new_value.
     * @tparam ParamType Type of the paramater.
     * @param full_name The full_name of the parameter to set.
     * @param new_value The new value of the parameter.
     */
    template <typename ParamType>
    void SetGlobalParameter(const std::string& full_name, const ParamType& new_value);

    /**
     * @brief Propagates the values of the global parameters to a specified node.
     * @param node Node which has parameters to update. 
     */
    void PropagateParameters(rclcpp::Node* node);

private:
    rclcpp::Node* node_;
    auto parameters_client = std::make_shared<rclcpp::AsyncParametersClient>(node_);
}


} // namespace params
