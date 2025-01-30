#pragma once

#include <rclcpp/rclcpp.hpp>

#include "rj_param_utils/ros2_param_provider.hpp"

namespace params {

/**
 *
 */
class LocalROS2ParamProvider : public BaseROS2ParamProvider {
public:
    explicit LocalROS2ParamProvider(rclcpp::Node* node, const std::string& module);

protected:
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

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr callback_handle_;
};

}  // namespace params