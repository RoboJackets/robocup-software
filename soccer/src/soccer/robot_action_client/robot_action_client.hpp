#pragma once

#include <functional>
#include <future>
#include <memory>
#include <string>
#include <sstream>

#include <spdlog/spdlog.h>
#include "rj_msgs/action/robot_move.hpp"

#include <rj_utils/logging.hpp>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

namespace robot_action_client
{
class RobotActionClient : public rclcpp::Node
{
public:
  using RobotMove = rj_msgs::action::RobotMove;
  using GoalHandleRobotMove = rclcpp_action::ClientGoalHandle<RobotMove>;

  RobotActionClient();

  void send_goal();

private:
  rclcpp_action::Client<RobotMove>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(std::shared_future<GoalHandleRobotMove::SharedPtr> future);
  void feedback_callback(
    GoalHandleRobotMove::SharedPtr,
    const std::shared_ptr<const RobotMove::Feedback> feedback);

  void result_callback(const GoalHandleRobotMove::WrappedResult & result);
};  // class RobotActionClient

}  // namespace robot_action_client
