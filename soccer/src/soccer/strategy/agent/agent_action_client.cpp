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
        "vision_filter/world_state", 1,
        [this](rj_msgs::msg::WorldState::SharedPtr msg) { world_state_callback(msg); });

    coach_state_sub_ = create_subscription<rj_msgs::msg::CoachState>(
        "strategy/coach_state", 1,
        [this](rj_msgs::msg::CoachState::SharedPtr msg) { coach_state_callback(msg); });

    // TODO: link to planning hz
    // TODO: change this once coach node merged
    int hz = 120;
    get_task_timer_ = create_wall_timer(std::chrono::milliseconds(1000 / hz),
                                        std::bind(&AgentActionClient::get_task, this));
}

void AgentActionClient::world_state_callback(rj_msgs::msg::WorldState::SharedPtr msg) {
    if (current_position_ == nullptr) {
        return;
    }

    WorldState world_state = rj_convert::convert_from_ros(*msg);
    current_position_->update_world_state(world_state);
}

void AgentActionClient::coach_state_callback(rj_msgs::msg::CoachState::SharedPtr msg) {
    if (current_position_ == nullptr) {
        return;
    }

    current_position_->update_coach_state(*msg);
}

void AgentActionClient::get_task() {
    // TODO: change this default to defense? or NOP?
    // TODO: move this once coach node merged
    if (current_position_ == nullptr) {
        current_position_ = std::make_unique<Offense>(2);
    }

    auto task = current_position_->get_task();
    if (task != last_task_) {
        last_task_ = task;
        send_new_goal();
    }
}

/* void AgentActionClient::cancel_last_goal() { */
/*     if (client_ptr_ == nullptr || last_goal_handle_ == nullptr) { */
/*         return; */
/*     } */
/*     using namespace std::placeholders; */
/*     auto cancel_future = client_ptr_->async_cancel_goal( */
/*         last_goal_handle_, std::bind(&AgentActionClient::cancel_goal_callback, this, _1)); */
/*     // TODO: do something with this future? */
/* } */

/* void AgentActionClient::cancel_goal_callback( */
/*     rclcpp_action::Client<RobotMove>::CancelResponse::SharedPtr) { */
/*     // TODO: worry about the cancelresponse? */
/*     // TODO: would this generate race cond? */
/*     last_goal_handle_ = nullptr; */
/* } */

void AgentActionClient::send_new_goal() {
    using namespace std::placeholders;

    if (!client_ptr_->wait_for_action_server()) {
        SPDLOG_ERROR("Action server not available after waiting");
        rclcpp::shutdown();
    }

    auto goal_msg = RobotMove::Goal();
    goal_msg.robot_intent = last_task_;

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
    }
}

void AgentActionClient::feedback_callback(
    GoalHandleRobotMove::SharedPtr, const std::shared_ptr<const RobotMove::Feedback> feedback) {
    double time_left = rj_convert::convert_from_ros(feedback->time_left).count();
    current_position_->tell_time_left(time_left);
}

void AgentActionClient::result_callback(const GoalHandleRobotMove::WrappedResult& result) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            // TODO: handle other return codes
            current_position_->tell_is_done();
            break;
        case rclcpp_action::ResultCode::ABORTED:
            return;
        case rclcpp_action::ResultCode::CANCELED:
            return;
        default:
            return;
    }
    /* SPDLOG_INFO("Result received: {}", result.result->is_done); */
}

}  // namespace strategy
