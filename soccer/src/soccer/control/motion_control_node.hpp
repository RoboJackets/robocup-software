#pragma once

#include <vector>

#include <rj_constants/constants.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <rj_param_utils/ros2_local_param_provider.hpp>
#include <rj_topic_utils/async_message_queue.hpp>

#include "motion_control.hpp"
#include "motion_setpoint.hpp"
#include "node.hpp"

namespace control {

/**
 * Handles control control for all robots. Calling this once will run control
 * control on all robots.
 */
class MotionControlNode : public rclcpp::Node {
public:
    explicit MotionControlNode();

private:
    ::params::LocalROS2ParamProvider param_provider_;
    std::vector<MotionControl> controllers_{};
};

}  // namespace control