#pragma once

#include <rj_topic_utils/async_message_queue.hpp>

#include <rj_constants/constants.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <rj_param_utils/ros2_param_provider.hpp>
#include <vector>

#include "motion_setpoint.hpp"
#include "motion_control.hpp"
#include "manipulator_control.hpp"
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
    ::params::ROS2ParamProvider param_provider_;
    std::vector<MotionControl> controllers_{};
    std::vector<ManipulatorControl> manipulators_{};
};

} // namespace control