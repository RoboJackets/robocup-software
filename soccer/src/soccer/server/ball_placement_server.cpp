#include "ball_placement_server.hpp"

#include <spdlog/spdlog.h>

namespace server {
using BallPlacement = rj_msgs::action::BallPlacement;
using GoalHandleBallPlacement = rclcpp_action::ServerGoalHandle<BallPlacement>;
BallPlacementServer ::BallPlacementServer(const rclcpp::NodeOptions& options)
    : Node("ball_placement_server", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<BallPlacement>(
        this->get_node_base_interface(), this->get_node_clock_interface(),
        this->get_node_logging_interface(), this->get_node_waitables_interface(), "ball_placement",
        std::bind(&BallPlacementServer::handle_goal, this, _1, _2),
        std::bind(&BallPlacementServer::handle_cancel, this, _1),
        std::bind(&BallPlacementServer::handle_accepted, this, _1));
}

rclcpp_action::GoalResponse BallPlacementServer ::handle_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const BallPlacement::Goal> goal) {
    (void)uuid;

    // NOTE: the goal_pt will be in our sim's coordinate frame (origin
    // at center of our goal, field oriented portrait mode), but will
    // be set by the ref in the league's frame (origin at center field,
    // field oriented landscape mode)
    rj_geometry::Point new_goal_pt = rj_geometry::Point(goal->goal_pt.x, goal->goal_pt.y);

    // reject duplicate goal pt requests from the client
    if (was_given_goal_pt_ && new_goal_pt.nearly_equals(curr_goal_pt_)) {
        SPDLOG_INFO("handle_goal: rejecting duplicate goal");
        return rclcpp_action::GoalResponse::REJECT;
        // or accept and execute new goal requests from the client
    } else {
        curr_goal_pt_ = new_goal_pt;
        was_given_goal_pt_ = true;
        SPDLOG_INFO("handle_goal: saved new_goal_pt: ({}, {})", curr_goal_pt_.x(),
                    curr_goal_pt_.y());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
}

rclcpp_action::CancelResponse BallPlacementServer ::handle_cancel(
    const std::shared_ptr<GoalHandleBallPlacement> goal_handle) {
    (void)goal_handle;
    was_given_goal_pt_ = false;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void BallPlacementServer ::handle_accepted(
    const std::shared_ptr<GoalHandleBallPlacement> goal_handle) {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    using namespace std::placeholders;
    std::thread{std::bind(&BallPlacementServer::execute, this, _1), goal_handle}.detach();
}

void BallPlacementServer ::execute(const std::shared_ptr<GoalHandleBallPlacement> goal_handle) {
    std::shared_ptr<const BallPlacement::Goal> goal = goal_handle->get_goal();

    while (true) {
        // TODO: plan here, change while loop condition if needed
        std::shared_ptr<BallPlacement::Result> result = std::make_shared<BallPlacement::Result>();

        std::shared_ptr<BallPlacement::Feedback> feedback =
            std::make_shared<BallPlacement::Feedback>();
        goal_handle->publish_feedback(feedback);

        if (goal_handle->is_canceling()) {
            result->is_done = true;
            goal_handle->canceled(result);
            return;
        }

        if (rclcpp::ok()) {
            result->is_done = true;
            goal_handle->succeed(result);
            /* std::cout << "Server succeeded!" << std::endl; */

            // TODO: on completion of goal, reset was_given_goal_pt_ state
            /* was_given_goal_pt_ = false; */
            break;
        }
    }
}
}  // namespace server
