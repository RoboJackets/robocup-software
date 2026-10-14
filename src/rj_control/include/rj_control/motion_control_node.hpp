#pragma once

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <rj_common/control/motion_setpoint.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <rj_utils/logging.hpp>

#include "rj_control/motion_control.hpp"

namespace control {

/**
 * Handles control control for all robots. Calling this once will run control
 * control on all robots.
 */
class MotionControlNode : public rclcpp::Node {
public:
    explicit MotionControlNode();

private:
    MotionControlLimits limits_{};
    std::vector<std::unique_ptr<MotionControl>> controllers_{};
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

}  // namespace control
