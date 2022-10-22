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

    world_state_sub_ = create_subscription<rj_msgs::msg::WorldState>(
        "vision_filter/world_state", 1, [this](rj_msgs::msg::WorldState::SharedPtr msg) {
            /*
            auto ball_state = rj_convert::convert_from_ros(msg->ball);
            if (spin_kick_detector(ball_state.position)) {
                send();
            }
            */
        });

    // TODO: move this once coach node merged
    current_position_ = std::make_unique<Goalie>();

    // TODO: link to planning hz
    // TODO: change this once coach node merged
    //
    // TODO: if timer is too fast, planner crashes, any fix?
    int hz = 2;
    get_task_timer_ = create_wall_timer(std::chrono::milliseconds(1000 / hz),
                                        std::bind(&AgentActionClient::get_task, this));
}

void AgentActionClient::get_task() {
    // TODO: change this default to defense? or NOP?
    if (current_position_ == nullptr) {
        current_position_ = std::make_unique<Goalie>();
    }

    auto task = current_position_->get_task();
    if (task != latest_task_) {
        latest_task_ = task;
        send_goal();
    }
}

void AgentActionClient::send_goal() {
    SPDLOG_INFO("latest_task_: {}", latest_task_.robot_id);

    using namespace std::placeholders;

    if (!client_ptr_->wait_for_action_server()) {
        SPDLOG_ERROR("Action server not available after waiting");
        rclcpp::shutdown();
    }

    auto goal_msg = RobotMove::Goal();
    goal_msg.robot_intent = latest_task_;

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
        current_position_->tell_goal_canceled();
        SPDLOG_ERROR("Goal was rejected by server");
    } else {
        SPDLOG_INFO("Goal accepted by server, waiting for result");
    }
}

void AgentActionClient::feedback_callback(
    GoalHandleRobotMove::SharedPtr, const std::shared_ptr<const RobotMove::Feedback> feedback) {
    double time_left = rj_convert::convert_from_ros(feedback->time_left).count();
    // SPDLOG_INFO("Time left: {}", time_left);
    current_position_->tell_time_left(time_left);
}

void AgentActionClient::result_callback(const GoalHandleRobotMove::WrappedResult& result) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            // TODO: handle other return codes
            current_position_->tell_is_done();
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
