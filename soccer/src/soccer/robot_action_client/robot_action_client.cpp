#include "robot_action_client.hpp"
// #include "planning/planner/motion_command.hpp"
#include <rj_msgs/msg/empty_motion_command.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry/geometry_conversions.hpp>

namespace robot_action_client
{
  using RobotMove = rj_msgs::action::RobotMove;
  using GoalHandleRobotMove = rclcpp_action::ClientGoalHandle<RobotMove>;

  RobotActionClient::RobotActionClient()
    : rclcpp::Node("robot_action_client_node",  rclcpp::NodeOptions{}
                                    .automatically_declare_parameters_from_overrides(true)
                                    .allow_undeclared_parameters(true))
  {
    this->client_ptr_ = rclcpp_action::create_client<RobotMove>(
      this,
      "robot_move");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&RobotActionClient::send_goal, this));
  }

  void RobotActionClient::send_goal()
  {
    using namespace std::placeholders;

    if (!this->client_ptr_->wait_for_action_server()) {
      SPDLOG_ERROR("Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = RobotMove::Goal();

    rj_msgs::msg::RobotIntent intent;
    intent.robot_id = 2;
    auto ptmc = rj_msgs::msg::PathTargetMotionCommand{};
    auto pt = rj_geometry::Point(2.0, 3.0);
    ptmc.target.position = rj_convert::convert_to_ros(pt);
    intent.motion_command.path_target_command = {ptmc};
    goal_msg.robot_intent = intent;

    SPDLOG_ERROR("Sending goal");

    auto send_goal_options = rclcpp_action::Client<RobotMove>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&RobotActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&RobotActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&RobotActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

  void RobotActionClient::goal_response_callback(std::shared_future<GoalHandleRobotMove::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      SPDLOG_ERROR("Goal was rejected by server");
    } else {
      SPDLOG_INFO("Goal accepted by server, waiting for result");
    }
  }

  void RobotActionClient::feedback_callback(
    GoalHandleRobotMove::SharedPtr,
    const std::shared_ptr<const RobotMove::Feedback> feedback)
  {
      // TODO: fill in feedback from server-side
  }

  void RobotActionClient::result_callback(const GoalHandleRobotMove::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        SPDLOG_ERROR("Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        SPDLOG_ERROR("Goal was canceled");
        return;
      default:
        SPDLOG_ERROR("Unknown result code");
        return;
    }
    SPDLOG_INFO("Result received: {}", result.result->is_done);
  }

}  // namespace robot_action_client
