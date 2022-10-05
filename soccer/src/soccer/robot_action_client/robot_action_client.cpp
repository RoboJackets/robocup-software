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

  RobotActionClient()
    : Node("planner",  rclcpp::NodeOptions{}
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

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server()) {
      SPDLOG_ERROR("Action server not available after waiting");
      rclcpp::shutdown();
    }

    auto goal_msg = RobotMove::Goal();
    rj_msgs::msg::RobotIntent intent;
    intent.robot_id = 2;

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

private:
  rclcpp_action::Client<RobotMove>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(std::shared_future<GoalHandleRobotMove::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      SPDLOG_ERROR("Goal was rejected by server");
    } else {
      SPDLOG_INFO("Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleRobotMove::SharedPtr,
    const std::shared_ptr<const RobotMove::Feedback> feedback)
  {
      // TODO: fill in feedback from server-side
  }

  void result_callback(const GoalHandleRobotMove::WrappedResult & result)
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
};  // class RobotActionClient

}  // namespace robot_action_client

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto robot_action_client_node = std::make_shared<robot_action_client::RobotActionClient>();
    /* start_global_param_provider(planner.get(), kGlobalParamServerNode); */
    rclcpp::spin(robot_action_client_node);
}
