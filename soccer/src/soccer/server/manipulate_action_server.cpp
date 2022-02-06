#include "manipulate_action_server.hpp"

namespace server {
using Manipulate = rj_msgs::action::Manipulate;
using GoalHandleManipulate = rclcpp_action::ServerGoalHandle<Manipulate>;
ManipulateActionServer ::ManipulateActionServer(const rclcpp::NodeOptions& options)
    : Node("manipulate_action_server", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Manipulate>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "manipulate", std::bind(&ManipulateActionServer::handle_goal, this, _1, _2),
        std::bind(&ManipulateActionServer::handle_cancel, this, _1),
        std::bind(&ManipulateActionServer::handle_accepted, this, _1));

    this->intent_pubs_.reserve(kNumShells);
    this->robot_statuses_.reserve(kNumShells);

    for (size_t i = 0; i < kNumShells; i++) {
        intent_pubs_.emplace_back(this->create_publisher<RobotIntent>(
            move_action_server::topics::robot_intent_pub(i), rclcpp::QoS(1).transient_local()));

        this->create_subscription<rj_msgs::msg::RobotStatus>(
            radio::topics::robot_status_pub(i), rclcpp::QoS(1),
            [this, i](rj_msgs::msg::RobotStatus::SharedPtr status) {  // NOLINT
                // TODO: why does this not work here?
                // RobotStatus status_ = *status;
                // this->robot_statuses_[i] = rj_convert::convert_from_ros(status_);
            });
    }
}

rclcpp_action::GoalResponse ManipulateActionServer ::handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                           std::shared_ptr<const Manipulate::Goal> goal) {
    (void)uuid;

    int robot_id = goal->server_intent.robot_id;
    RobotIntent robot_intent = goal->server_intent.intent;

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse ManipulateActionServer ::handle_cancel(
    const std::shared_ptr<GoalHandleManipulate> goal_handle) {
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void ManipulateActionServer ::handle_accepted(const std::shared_ptr<GoalHandleManipulate> goal_handle) {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    using namespace std::placeholders;
    std::thread{std::bind(&ManipulateActionServer::execute, this, _1), goal_handle}.detach();
}

void ManipulateActionServer ::execute(const std::shared_ptr<GoalHandleManipulate> goal_handle) {
    std::shared_ptr<const Manipulate::Goal> goal = goal_handle->get_goal();
    rj_msgs::msg::ServerIntent server_intent = goal->server_intent;
    rj_msgs::msg::RobotIntent robot_intent = server_intent.intent;

    int robot_id = server_intent.robot_id;

    std::shared_ptr<Manipulate::Result> result = std::make_shared<Manipulate::Result>();

    this->intent_pubs_[robot_id]->publish(robot_intent);

    /*
    std::shared_ptr<Manipulate::Feedback> feedback = std::make_shared<Manipulate::Feedback>();
    goal_handle->publish_feedback(feedback);
     */

    if (goal_handle->is_canceling()) {
        result->is_done = true;
        goal_handle->canceled(result);
        return;
    }

    if (rclcpp::ok()) {
        result->is_done = true;
        goal_handle->succeed(result);
    }
}
}  // namespace server
