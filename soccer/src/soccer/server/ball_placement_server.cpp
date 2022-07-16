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
    return rclcpp_action::CancelResponse::ACCEPT;
}

void BallPlacementServer::handle_accepted(
    const std::shared_ptr<GoalHandleBallPlacement> goal_handle) {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    using namespace std::placeholders;
    current_state_ = INIT;
    std::thread{std::bind(&BallPlacementServer::execute, this, _1), goal_handle}.detach();
}

void BallPlacementServer::execute(const std::shared_ptr<GoalHandleBallPlacement> goal_handle) {
    // TODO: make this cancel on HALT/STOP update somehow

    // rate-limit loop to 1ms per iteration
    rclcpp::Rate loop_rate(1);

    // create ptrs to Goal, Result objects per ActionServer API
    std::shared_ptr<const BallPlacement::Goal> goal = goal_handle->get_goal();
    std::shared_ptr<BallPlacement::Result> result = std::make_shared<BallPlacement::Result>();

    // get goal_pt (won't change unless goal changes)
    rj_geometry::Point goal_pt = rj_convert::convert_from_ros(goal->goal_pt);

    bool not_done_yet = true;
    while (not_done_yet) {
        // if the ActionClient is trying to cancel the goal, cancel it, terminate early
        if (goal_handle->is_canceling()) {
            result->is_done = false;
            goal_handle->canceled(result);
            return;
        }

        // publish feedback (necessary even if feedback is empty)
        std::shared_ptr<BallPlacement::Feedback> feedback =
            std::make_shared<BallPlacement::Feedback>();
        feedback->est_time_remaining = rclcpp::Duration(1.0);
        goal_handle->publish_feedback(feedback);

        // otherwise, follow existing state machine to completion
        switch (current_state_) {
            case INIT: {
                SPDLOG_INFO("INIT");
                // find ball->goal_pt vector
                BallState ball = last_world_state_.ball;
                rj_geometry::Point ball_to_goal_dir = (goal_pt - ball.position).norm();

                // make other robots move out of vector path?? (check rules on if allowed)
                // for now if there is a robot in the way, ignore it
                // (TODO: later can dribble ball to side and then pass)

                // how many robot_rads off the exact points each robot should sit (min 1.0)
                const double slack_const = 1.5;

                // set desired robot 0 pt to kicker spot (slightly behind ball)
                robot_0_start_pt_ = ball.position - ball_to_goal_dir * (slack_const * kRobotRadius);
                // set desired robot 1 pt to receiver spot (slightly behind goal_pt)
                robot_1_start_pt_ = goal_pt + ball_to_goal_dir * (slack_const * kRobotRadius);

                // change state
                current_state_ = MOVING_TO_SPOTS;
                break;
            }
            case MOVING_TO_SPOTS: {
                SPDLOG_INFO("MOVING_TO_SPOTS");
                // move robot 0 to behind ball, facing robot 1
                RobotIntent robot_intent_0;
                planning::PathTargetCommand pt_0{
                    planning::LinearMotionInstant(robot_0_start_pt_, rj_geometry::Point(0.0, 0.0)),
                    planning::TargetFacePoint{robot_1_start_pt_}};
                robot_intent_0.motion_command = pt_0;

                // move robot 1 to right behind ball placement pt, facing robot 0
                RobotIntent robot_intent_1;
                planning::PathTargetCommand pt_1{
                    planning::LinearMotionInstant(robot_1_start_pt_, rj_geometry::Point(0.0, 0.0)),
                    planning::TargetFacePoint{robot_0_start_pt_}};
                robot_intent_1.motion_command = pt_1;

                // publish both robot intents, await completion
                robot_intent_0.is_active = true;
                robot_intent_0_pub_->publish(rj_convert::convert_to_ros(robot_intent_0));

                robot_intent_1.is_active = true;
                robot_intent_1_pub_->publish(rj_convert::convert_to_ros(robot_intent_1));

                // when both moves are done, wipe robot intents, go to next state
                if (last_is_done_0_ && last_is_done_1_) {
                    RobotIntent stop_intent;
                    stop_intent.is_active = false;
                    robot_intent_0_pub_->publish(rj_convert::convert_to_ros(stop_intent));
                    robot_intent_1_pub_->publish(rj_convert::convert_to_ros(stop_intent));
                    current_state_ = PASSING;
                }
                break;
            }
            case PASSING: {
                SPDLOG_INFO("PASSING");
                BallState ball = last_world_state_.ball;
                if (!has_kicked_) {
                    // line kick robot 0 -> 1
                    RobotIntent intent;
                    intent.motion_command = planning::LineKickCommand{goal_pt};
                    // TODO: these should really be bound to the KickCommand, not the RobotIntent
                    intent.kick_speed = 2.0;  // TODO: base kick speed on dist to target
                    intent.shoot_mode = RobotIntent::ShootMode::KICK;
                    intent.trigger_mode = RobotIntent::TriggerMode::IMMEDIATE;
                    intent.is_active = true;

                    // robot 0 should only kick if ball is stopped
                    if (ball.velocity.mag() > 0.3) {
                        intent.is_active = false;
                    }

                    robot_intent_0_pub_->publish(rj_convert::convert_to_ros(intent));
                }

                // should turn on receiver's dribbler, and have it drive backwards
                // to stop the ball right on the point, but we have no planner
                // for "backstop the ball onto this exact pt". should write one
                // for better passing. or at least have a dribbler-on planner
                //
                // instead we have ADJUSTING, which calls collect planner to
                // try and get the ball over the pt
                //
                // TODO(Kevin): write dribbler-on planner
                // TODO(Kevin): write true passing receive planner (where it backstops to a point)

                // TODO(Kevin): figure out why line kick's is_done() doesn't work
                /*
                SPDLOG_INFO("is robot 0 done? {}", last_is_done_0_);
                // when robot 0 is done kicking, wipe intent, go to next state
                if (last_is_done_0_) {
                    RobotIntent stop_intent;
                    stop_intent.is_active = false;
                    robot_intent_0_pub_->publish(rj_convert::convert_to_ros(stop_intent));
                    current_state_ = DONE;
                }
                */

                // when the ball is close to goal, go to next state
                // TODO: (this could deadlock if the line kick is bad and the ball
                // doesn't end up where it should...)
                if (ball.position.nearly_equals(goal_pt, BALL_PLACEMENT_TOLERANCE_)) {
                    RobotIntent stop_intent;
                    stop_intent.is_active = false;
                    robot_intent_0_pub_->publish(rj_convert::convert_to_ros(stop_intent));
                    current_state_ = RETREAT;
                }
                break;
            }
            case RETREAT: {
                SPDLOG_INFO("RETREAT");

                rj_geometry::Point ball_pt = last_world_state_.ball.position;
                if (!end_pts_sent_) {
                    // move robots >=0.5 m in dir away from ball (per rules)
                    double retreat_dist = 0.1 + 0.5 + kRobotRadius;  // m

                    // in reality this should be a signal to move back to normal
                    // STOP behavior, but we have circumvented the rest of gameplay
                    // so that won't be feasible
                    rj_geometry::Point robot_0_pt =
                        last_world_state_.our_robots.at(0).pose.position();
                    rj_geometry::Point ball_robot_0_dir = (robot_0_pt - ball_pt).norm();
                    robot_0_end_pt_ = robot_0_pt + ball_robot_0_dir * retreat_dist;

                    rj_geometry::Point robot_1_pt =
                        last_world_state_.our_robots.at(1).pose.position();
                    rj_geometry::Point ball_robot_1_dir = (robot_1_pt - ball_pt).norm();
                    robot_1_end_pt_ = robot_1_pt + ball_robot_1_dir * retreat_dist;

                    RobotIntent robot_intent_0;
                    planning::PathTargetCommand pt_0{
                        planning::LinearMotionInstant(robot_0_end_pt_,
                                                      rj_geometry::Point(0.0, 0.0)),
                        planning::TargetFacePoint{goal_pt}};
                    robot_intent_0.motion_command = pt_0;

                    // move robot 1 to right behind ball placement pt, facing robot 0
                    RobotIntent robot_intent_1;
                    planning::PathTargetCommand pt_1{
                        planning::LinearMotionInstant(robot_1_end_pt_,
                                                      rj_geometry::Point(0.0, 0.0)),
                        planning::TargetFacePoint{goal_pt}};
                    robot_intent_1.motion_command = pt_1;

                    // publish both robot intents, await completion
                    robot_intent_0.is_active = true;
                    robot_intent_0_pub_->publish(rj_convert::convert_to_ros(robot_intent_0));

                    robot_intent_1.is_active = true;
                    robot_intent_1_pub_->publish(rj_convert::convert_to_ros(robot_intent_1));
                    end_pts_sent_ = true;
                }

                // when both moves are done, wipe robot intents, go to next state
                if (end_pts_sent_ && last_is_done_0_ && last_is_done_1_) {
                    RobotIntent stop_intent;
                    stop_intent.is_active = false;
                    robot_intent_0_pub_->publish(rj_convert::convert_to_ros(stop_intent));
                    robot_intent_1_pub_->publish(rj_convert::convert_to_ros(stop_intent));
                    current_state_ = DONE;
                }
                break;
            }
            case DONE: {
                SPDLOG_INFO("DONE");
                not_done_yet = false;
                break;
            }
        }

        // rate-limit loop
        loop_rate.sleep();
    }

    SPDLOG_INFO("not_done_yet {}", not_done_yet);
    // after done, send success result
    if (rclcpp::ok()) {
        result->is_done = true;
        goal_handle->succeed(result);
        SPDLOG_INFO("Server succeeded!");

        // on completion of goal, reset was_given_goal_pt_ state
        was_given_goal_pt_ = false;

        // reset FSM also for next ball placement
        current_state_ = INIT;
    }
}

}  // namespace server
