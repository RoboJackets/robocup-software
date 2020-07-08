#pragma once

#include <rj_param_utils/param.h>

namespace params {
/**
 * An implementation of ParamProvider that integrates with the ROS2 parameter
 * system.
 */
class ROS2ParamProvider : public ::params::internal::ParamProvider {
public:
    explicit ROS2ParamProvider(rclcpp::Node* node);

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

    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
        callback_handle_;
};
}  // namespace params
