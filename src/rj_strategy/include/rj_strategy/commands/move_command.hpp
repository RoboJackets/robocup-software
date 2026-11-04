#pragma once
#include <rclcpp_action/rclcpp_action.hpp>

#include <rj_common/field_dimensions.hpp>
#include <rj_common/game_state.hpp>
#include <rj_common/robot_intent.hpp>
#include <rj_common/ros_debug_drawer.hpp>
#include <rj_common/time.hpp>
#include <rj_common/world_state.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/action/robot_move.hpp>
#include <rj_msgs/msg/alive_robots.hpp>
#include <rj_msgs/msg/motion_command.hpp>

#include "rj_strategy/command.hpp"
#include "rj_strategy/robot.hpp"

namespace commands {

class MoveCommand : public Command {
public:
    MoveCommand(std::shared_ptr<Robot> robot, planning::MotionCommand motion_command);
    void initialize() override;
    bool is_done() override;

private:
    planning::MotionCommand motion_command;

    std::shared_ptr<Robot> robot;
};

}  // namespace commands