/** @file */
#pragma once

#include <rclcpp/node.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rj_param_utils/param.hpp>
#include <rj_param_utils/ros2_param_provider.hpp>

namespace params {

constexpr auto kGlobalModule = "global_params";

/**
 * Implementation of a global parameter provider used by every node.
 *
 * Important: only one global param provider should exist per program instance (not per-node).
 */
class ROS2GlobalParamProvider : public BaseROS2ParamProvider {
public:
    /**
     * Initialize a global param provider.
     *
     * @param node the node to use for the global parameter provider. This node should NOT be
     * associated with an executor.
     * @param module the string name of the global param module. Usually kGlobalModule.
     * @param global_node the string name of the global parameter server node.
     */
    explicit ROS2GlobalParamProvider(rclcpp::Node* node, const std::string& module,
                                     const std::string& global_node);

private:
    rclcpp::AsyncParametersClient::SharedPtr params_client_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr params_client_sub_;
};

}  // namespace params
