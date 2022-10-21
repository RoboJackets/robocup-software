#include "agent_action_client.hpp"

namespace strategy {
using RobotMove = rj_msgs::action::RobotMove;
using GoalHandleRobotMove = rclcpp_action::ClientGoalHandle<RobotMove>;

AgentActionClient::AgentActionClient()
    : rclcpp::Node("agent_action_client_node",
                   rclcpp::NodeOptions{}
                       .automatically_declare_parameters_from_overrides(true)
                       .allow_undeclared_parameters(true)) {
    // create a ptr to ActionClient
    client_ptr_ = rclcpp_action::create_client<RobotMove>(this, "robot_move");

    // create a Position class to delegate most decision-making to
    // (Strategy Design Pattern)
    // TODO: move this once coach node merged
    current_position_ = Position();

    // TODO: rm this once coach node merged
    timer_ = create_wall_timer(std::chrono::milliseconds(500),
                               std::bind(&AgentActionClient::send_goal, this));
}

void AgentActionClient::send_goal() {
    // for now, only send one goal
    timer_->cancel();

    using namespace std::placeholders;

    if (!client_ptr_->wait_for_action_server()) {
        SPDLOG_ERROR("Action server not available after waiting");
        rclcpp::shutdown();
    }

    auto goal_msg = RobotMove::Goal();
    goal_msg.robot_intent = current_position_.get_task();

    SPDLOG_ERROR("Sending goal");

    auto send_goal_options = rclcpp_action::Client<RobotMove>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&AgentActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&AgentActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&AgentActionClient::result_callback, this, _1);
    client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void AgentActionClient::goal_response_callback(
    std::shared_future<GoalHandleRobotMove::SharedPtr> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
        SPDLOG_ERROR("Goal was rejected by server");
    } else {
        SPDLOG_INFO("Goal accepted by server, waiting for result");
    }
}

void AgentActionClient::feedback_callback(
    GoalHandleRobotMove::SharedPtr, const std::shared_ptr<const RobotMove::Feedback> feedback) {
    auto time_left = rj_convert::convert_from_ros(feedback->time_left).count();
    SPDLOG_INFO("Time left: {}", time_left);
}

void AgentActionClient::result_callback(const GoalHandleRobotMove::WrappedResult& result) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            SPDLOG_ERROR("Goal succeeded!");
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

}  // namespace strategy
