#pragma once

#include <stdexcept>
#include <string>

#include <rclcpp/node.hpp>

namespace rj_utils {

/**
 * @brief Declares a parameter with no default and returns its value from the
 * node's params file (sim_params.yaml or real_params.yaml).
 *
 * The parameter is declared with its type and description, so it shows up in
 * `ros2 param list/describe`. There is intentionally no fallback: if the
 * params file does not set it, or sets it with the wrong type, this throws at
 * startup naming the node and key.
 *
 * The value is read once, at startup. `ros2 param set` does not change node
 * behavior; to change a value, edit the YAML and restart the node.
 */
template <typename T>
T declare_required_param(rclcpp::Node& node, const std::string& name,
                         const std::string& description) {
    const std::string node_name = node.get_fully_qualified_name();
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;

    rclcpp::ParameterValue value;
    try {
        value = node.declare_parameter(name, rclcpp::ParameterValue(T{}).get_type(), descriptor);
    } catch (const rclcpp::exceptions::InvalidParameterTypeException& e) {
        throw std::runtime_error("Parameter '" + name + "' on node '" + node_name +
                                 "' has the wrong type in the params file: " + e.what());
    }

    // Humble does not throw when a typed parameter has no value, so check here.
    if (value.get_type() == rclcpp::ParameterType::PARAMETER_NOT_SET) {
        throw std::runtime_error("Missing required parameter '" + name + "' on node '" + node_name +
                                 "'. Add it under '" + node_name +
                                 "' in sim_params.yaml and real_params.yaml.");
    }
    return value.get<T>();
}

}  // namespace rj_utils
