/** @file */
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <rj_param_utils/param.hpp>
#include <rj_param_utils/ros2_param_provider.hpp>


namespace params {
/**
 * Implementation of a global parameter provider used by every node.
 */

class ROS2GlobalParamProvider : public ::params::ROS2ParamProvider {
public:
    explicit ROS2GlobalParamProvider(rclcpp::Node* node, const rclcpp::Node* global_node); 

private:
    rclcpp::Node* node_;
    rclcpp::Node* global_node_;
    rclcpp::AsyncParameterClient::SharedPtr params_client_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr params_client_sub_;
    
    /**
     * @brief Updates the parameters in the global params node.
     */
    void UpdateGlobalParams();
}

} // namespace params
