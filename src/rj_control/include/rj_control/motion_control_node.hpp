#pragma once

#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <rj_common/control/motion_setpoint.hpp>
#include <rj_constants/constants.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <rj_topic_utils/async_message_queue.hpp>
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
    std::vector<MotionControl> controllers_{};
};

}  // namespace control