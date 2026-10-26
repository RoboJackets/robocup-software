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

namespace commands {

class MoveCommand : public Command {
public:
    using RobotMove = rj_msgs::action::RobotMove;
    using GoalHandleRobotMove = rclcpp_action::ClientGoalHandle<RobotMove>;
    MoveCommand(::Robot robot, planning::MotionCommand motion_command,
                rclcpp_action::Client<RobotMove>::SharedPtr client_ptr);
    void initialize() override;
    bool is_done() override;

private:
    planning::MotionCommand motion_command;
    rclcpp_action::Client<RobotMove>::SharedPtr client_ptr;
    void result_callback(const GoalHandleRobotMove::WrappedResult& result);
    void goal_response_callback(GoalHandleRobotMove::SharedPtr goal_handle);
    bool finished;
};

}  // namespace commands