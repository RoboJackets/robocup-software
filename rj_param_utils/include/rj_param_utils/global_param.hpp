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
    explicit ROS2GlobalParamProvider(rclcpp::Node* node, const std::string& global_node); 

private:
    rclcpp::Node* node_;
    // const std::string& global_node_;
    rclcpp::AsyncParametersClient::SharedPtr params_client_;
    rclcpp::Subscription<rcl_interfaces::msg::ParameterEvent>::SharedPtr params_client_sub_;
};
} // namespace params
