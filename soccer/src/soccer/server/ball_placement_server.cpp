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

    // setup robot intent pubs
    robot_intent_0_pub_ = this->create_publisher<RobotIntent::Msg>(
        gameplay::topics::robot_intent_pub(0), rclcpp::QoS(1).transient_local());
    robot_intent_1_pub_ = this->create_publisher<RobotIntent::Msg>(
        gameplay::topics::robot_intent_pub(1), rclcpp::QoS(1).transient_local());

    // save latest is_done status every time subs are updated
    // TODO: macro for is_done topic name like other topics (see above)?
    is_done_0_sub_ = this->create_subscription<rj_msgs::msg::IsDone>(
        "planning/is_done/robot_0", rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::IsDone::SharedPtr msg) {  // NOLINT
            last_is_done_0_ = msg->is_done;
        });
    is_done_1_sub_ = this->create_subscription<rj_msgs::msg::IsDone>(
        "planning/is_done/robot_1", rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::IsDone::SharedPtr msg) {  // NOLINT
            last_is_done_1_ = msg->is_done;
        });

    // save latest world_state status every time sub is updated
    // TODO: does this have to be mutex locked or is that just planner node?
    world_state_sub_ = this->create_subscription<rj_msgs::msg::WorldState>(
        vision_filter::topics::kWorldStatePub, rclcpp::QoS(1),
        [this](rj_msgs::msg::WorldState::SharedPtr world_state) {  // NOLINT
            last_world_state_ = rj_convert::convert_from_ros(*world_state);
        });
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
    current_state_ = BallPlacementState::INIT;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void BallPlacementServer::handle_accepted(
    const std::shared_ptr<GoalHandleBallPlacement> goal_handle) {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    using namespace std::placeholders;
    std::thread{std::bind(&BallPlacementServer::execute, this, _1), goal_handle}.detach();
}

void BallPlacementServer::execute(const std::shared_ptr<GoalHandleBallPlacement> goal_handle) {
    // TODO: make this cancel on HALT/STOP update somehow

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
                // find ball->goal_pt vector
                BallState ball = last_world_state_.ball;
                rj_geometry::Point goal_pt = rj_convert::convert_from_ros(goal->goal_pt);
                rj_geometry::Point ball_to_goal_dir = (goal_pt - ball.position).norm();

                // make other robots move out of vector path?? (check rules on if allowed)
                // for now if there is a robot in the way, ignore it
                // (TODO: later can dribble ball to side and then pass)

                // how many robot_rads off the exact points each robot should sit (min 1.0)
                const double slack_const = 1.5;

                // set desired robot 0 pt to kicker spot (slightly behind ball)
                robot_0_spot_ = ball.position - ball_to_goal_dir * (slack_const * kRobotRadius);
                // set desired robot 1 pt to receiver spot (slightly behind goal_pt)
                robot_1_spot_ = goal_pt + ball_to_goal_dir * (slack_const * kRobotRadius);

                // change state
                current_state_ = MOVING_TO_SPOTS;
                break;
            }
            case MOVING_TO_SPOTS: {
                SPDLOG_INFO("MOVING_TO_SPOTS");
                // move robot 0 to behind ball, facing robot 1
                RobotIntent robot_intent_0;
                planning::PathTargetCommand pt_0{
                    planning::LinearMotionInstant(robot_0_spot_, rj_geometry::Point(0.0, 0.0)),
                    planning::TargetFacePoint{robot_1_spot_}};
                robot_intent_0.motion_command = pt_0;

                // move robot 1 to right behind ball placement pt, facing robot 0
                RobotIntent robot_intent_1;
                planning::PathTargetCommand pt_1{
                    planning::LinearMotionInstant(robot_1_spot_, rj_geometry::Point(0.0, 0.0)),
                    planning::TargetFacePoint{robot_0_spot_}};
                robot_intent_1.motion_command = pt_1;

                // publish both robot intents, await completion
                robot_intent_0.is_active = true;
                robot_intent_0_pub_->publish(rj_convert::convert_to_ros(robot_intent_0));

                robot_intent_1.is_active = true;
                robot_intent_1_pub_->publish(rj_convert::convert_to_ros(robot_intent_1));

                // when both moves are done, go to next state
                if (last_is_done_0_ && last_is_done_1_) {
                    current_state_ = PASSING;
                }
                break;
            }
            case PASSING: {
                SPDLOG_INFO("PASSING");
                // 2) line kick robot A->B + turn on B dribbler (maybe should consider doing the
                // backwards capture drive here?)
                break;
            }
            case ADJUSTING: {
                SPDLOG_INFO("ADJUSTING");
                /* 3) have B capture ball onto point if necessary (in case of bounce) */
                break;
            }
            case DONE: {
                // TODO : add another state to back up .5 m from the ball
                //   this is part of the rules
                SPDLOG_INFO("DONE");

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
