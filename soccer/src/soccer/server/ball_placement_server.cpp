#include "ball_placement_server.hpp"

#include <spdlog/spdlog.h>

namespace server {
using BallPlacement = rj_msgs::action::BallPlacement;
using GoalHandleBallPlacement = rclcpp_action::ServerGoalHandle<BallPlacement>;
BallPlacementServer::BallPlacementServer(const rclcpp::NodeOptions& options)
    : Node("ball_placement_server", options) {
    using namespace std::placeholders;

    // set up ActionServer + callbacks
    this->action_server_ = rclcpp_action::create_server<BallPlacement>(
        this->get_node_base_interface(), this->get_node_clock_interface(),
        this->get_node_logging_interface(), this->get_node_waitables_interface(), "ball_placement",
        std::bind(&BallPlacementServer::handle_goal, this, _1, _2),
        std::bind(&BallPlacementServer::handle_cancel, this, _1),
        std::bind(&BallPlacementServer::handle_accepted, this, _1));

    // set up robot planners
    /* robot_planner_0_ = std::make_shared<planning::PlannerForRobot>(0, this, &robot_trajectories_,
     * &shared_state_); */
    /* robot_planner_1_ = std::make_shared<planning::PlannerForRobot>(1, this, &robot_trajectories_,
     * &shared_state_); */

    robot_intent_0_pub_ = this->create_publisher<RobotIntent::Msg>(
        gameplay::topics::robot_intent_pub(0), rclcpp::QoS(1).transient_local());
    robot_intent_1_pub_ = this->create_publisher<RobotIntent::Msg>(
        gameplay::topics::robot_intent_pub(1), rclcpp::QoS(1).transient_local());

    // TODO: macro for is_done topic name like other topics (see above)?
    is_done_0_sub_ = this->create_subscription<rj_msgs::msg::IsDone>(
        "planning/is_done/robot_0", rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::IsDone::SharedPtr msg) { latest_is_done_0_ = msg->is_done; });
    is_done_1_sub_ = this->create_subscription<rj_msgs::msg::IsDone>(
        "planning/is_done/robot_1", rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::IsDone::SharedPtr msg) { latest_is_done_1_ = msg->is_done; });
}

rclcpp_action::GoalResponse BallPlacementServer::handle_goal(
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

rclcpp_action::CancelResponse BallPlacementServer::handle_cancel(
    const std::shared_ptr<GoalHandleBallPlacement> goal_handle) {
    (void)goal_handle;
    was_given_goal_pt_ = false;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void BallPlacementServer::handle_accepted(
    const std::shared_ptr<GoalHandleBallPlacement> goal_handle) {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    using namespace std::placeholders;
    std::thread{std::bind(&BallPlacementServer::execute, this, _1), goal_handle}.detach();
}

void BallPlacementServer::execute(const std::shared_ptr<GoalHandleBallPlacement> goal_handle) {
    // rate-limit loop to 1ms per iteration
    rclcpp::Rate loop_rate(1);

    // create ptrs to Goal, Result objects per ActionServer API
    std::shared_ptr<const BallPlacement::Goal> goal = goal_handle->get_goal();
    std::shared_ptr<BallPlacement::Result> result = std::make_shared<BallPlacement::Result>();

    while (true) {
        // if the ActionClient is trying to cancel the goal, cancel it, terminate early
        if (goal_handle->is_canceling()) {
            result->is_done = true;
            goal_handle->canceled(result);
            return;
        }

        // otherwise, follow existing state machine to completion
        switch (current_state_) {
            case INIT: {
                SPDLOG_INFO("INIT");
                // create and send robot request to relevant PlannerForRobot
                RobotIntent robot_intent_0;
                planning::PathTargetCommand path_target{
                    planning::LinearMotionInstant(rj_geometry::Point(1.0, 2.0),
                                                  rj_geometry::Point(0.0, 0.0)),
                    planning::TargetFacePoint{rj_geometry::Point(1.0, 2.0)}};

                robot_intent_0.motion_command = path_target;
                robot_intent_0.is_active = true;
                robot_intent_0_pub_->publish(rj_convert::convert_to_ros(robot_intent_0));

                if (latest_is_done_0_) {
                    current_state_ = MOVING_TO_SPOTS;
                }
                break;
            }
            case MOVING_TO_SPOTS: {
                SPDLOG_INFO("MOVING_TO_SPOTS");

                break;
            }
            case PASSING: {
                break;
            }
            case ADJUSTING: {
                break;
            }
            case DONE: {
                // when done, send success result
                if (rclcpp::ok()) {
                    result->is_done = true;
                    goal_handle->succeed(result);
                    std::cout << "Server succeeded!" << std::endl;

                    // TODO: on completion of goal, reset was_given_goal_pt_ state
                    /* was_given_goal_pt_ = false; */
                }
                break;
            }
            default: {
                // TODO: how to handle invalid state here?
                break;
            }
        }

        // TODO: how to populate feedback, what to put in?
        /* std::shared_ptr<BallPlacement::Feedback> feedback =
         * std::make_shared<BallPlacement::Feedback>(); */
        /* goal_handle->publish_feedback(feedback); */

        // rate-limit loop
        loop_rate.sleep();
    }
}

}  // namespace server
