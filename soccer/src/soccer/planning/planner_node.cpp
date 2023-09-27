#include "planner_node.hpp"

#include <boost/algorithm/string.hpp>
#include <spdlog/spdlog.h>

#include <rj_constants/topic_names.hpp>
#include <ros_debug_drawer.hpp>

#include "global_state.hpp"
#include "instant.hpp"
#include "planner_for_robot.hpp"
#include "planning/planner/collect_path_planner.hpp"
#include "planning/planner/escape_obstacles_path_planner.hpp"
#include "planning/planner/goalie_idle_path_planner.hpp"
#include "planning/planner/intercept_path_planner.hpp"
#include "planning/planner/line_kick_path_planner.hpp"

namespace planning {

using RobotMove = rj_msgs::action::RobotMove;
using GoalHandleRobotMove = rclcpp_action::ServerGoalHandle<RobotMove>;

PlannerNode::PlannerNode()
    : rclcpp::Node("planner", rclcpp::NodeOptions{}
                                  .automatically_declare_parameters_from_overrides(true)
                                  .allow_undeclared_parameters(true)),
      global_state_(this),
      param_provider_{this, kPlanningParamModule} {
    // for _1, _2 etc. below
    using namespace std::placeholders;

    // set up ActionServer + callbacks
    this->action_server_ = rclcpp_action::create_server<RobotMove>(
        this->get_node_base_interface(), this->get_node_clock_interface(),
        this->get_node_logging_interface(), this->get_node_waitables_interface(), "robot_move",
        std::bind(&PlannerNode::handle_goal, this, _1, _2),
        std::bind(&PlannerNode::handle_cancel, this, _1),
        std::bind(&PlannerNode::handle_accepted, this, _1));

    // set up PlannerForRobot objects
    robot_planners_.reserve(kNumShells);
    for (size_t i = 0; i < kNumShells; i++) {
        auto planner =
            std::make_unique<PlannerForRobot>(i, this, &robot_trajectories_, global_state_);
        robot_planners_.emplace_back(std::move(planner));
    }
}

rclcpp_action::GoalResponse PlannerNode::handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                     std::shared_ptr<const RobotMove::Goal> goal) {
    (void)uuid;
    auto delay = std::chrono::milliseconds(1000 / 60);
    rclcpp::Rate loop_rate(delay);

    // TODO(p-nayak): REJECT duplicate goal requests so we aren't constantly replanning them

    // planning::MotionCommand motion_command_ = goal->robot_intent.motion_command;

    int robot_id = goal->robot_intent.robot_id;
    auto& robot_task = server_task_states_.at(robot_id);
    auto& is_executing = robot_task.is_executing;
    auto& new_task_waiting_signal = robot_task.new_task_waiting_signal;
    while (is_executing) {
        new_task_waiting_signal = true;
        loop_rate.sleep();
    }
    new_task_waiting_signal = false;
    is_executing = true;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlannerNode::handle_cancel(
    const std::shared_ptr<GoalHandleRobotMove> goal_handle) {
    (void)goal_handle;
    std::shared_ptr<const RobotMove::Goal> goal = goal_handle->get_goal();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void PlannerNode::handle_accepted(const std::shared_ptr<GoalHandleRobotMove> goal_handle) {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    // execute() will block (loop) until completion (either success or canceled by client)
    using namespace std::placeholders;
    std::thread{std::bind(&PlannerNode::execute, this, _1), goal_handle}.detach();
}

void PlannerNode::execute(const std::shared_ptr<GoalHandleRobotMove> goal_handle) {
    // TODO(Kevin): rate-limit loop to whatever hz planning is limited to
    auto delay = std::chrono::milliseconds(1000 / 60);
    rclcpp::Rate loop_rate(delay);

    // create ptrs to Goal, Result objects per ActionServer API
    std::shared_ptr<const RobotMove::Goal> goal = goal_handle->get_goal();
    std::shared_ptr<RobotMove::Result> result = std::make_shared<RobotMove::Result>();

    // get correct PlannerForRobot object for this robot_id
    int robot_id = goal->robot_intent.robot_id;
    // reference to unique_ptr to avoid transferring ownership
    PlannerForRobot& my_robot_planner = *robot_planners_[robot_id];

    auto& robot_task = server_task_states_.at(robot_id);

    // loop until goal is done (SUCCEEDED or CANCELED)
    for (;;) {
        auto& new_task_ready = robot_task.new_task_waiting_signal;
        // check if there is a new goal
        if (new_task_ready) {
            result->is_done = false;
            goal_handle->abort(result);
            break;
        }

        // if the ActionClient is trying to cancel the goal, cancel it & terminate early
        if (goal_handle->is_canceling()) {
            result->is_done = false;
            goal_handle->canceled(result);
            break;
        }

        // pub Trajectory based on the RobotIntent
        my_robot_planner.execute_intent(rj_convert::convert_from_ros(goal->robot_intent));

        /*
        // TODO (PR #1970): fix TrajectoryCollection
        // send feedback
        std::shared_ptr<RobotMove::Feedback> feedback = std::make_shared<RobotMove::Feedback>();
        if (auto time_left = my_robot_planner.get_time_left()) {
            feedback->time_left = rj_convert::convert_to_ros(time_left.value());
            goal_handle->publish_feedback(feedback);
        }
        */

        // when done, tell client goal is done, break loop
        // TODO(p-nayak): when done, publish empty motion command to this robot's trajectory
        if (my_robot_planner.is_done()) {
            if (rclcpp::ok()) {
                result->is_done = true;
                goal_handle->succeed(result);
                break;
            }
        }
        loop_rate.sleep();
    }
    robot_task.is_executing = false;
}

void PlannerForRobot::plan_hypothetical_robot_path(
    const std::shared_ptr<rj_msgs::srv::PlanHypotheticalPath::Request>& request,
    std::shared_ptr<rj_msgs::srv::PlanHypotheticalPath::Response>& response) {
    /* const auto intent = rj_convert::convert_from_ros(request->intent); */
    /* auto plan_request = make_request(intent); */
    /* auto trajectory = safe_plan_for_robot(plan_request); */
    /* RJ::Seconds trajectory_duration = trajectory.duration(); */
    /* response->estimate = rj_convert::convert_to_ros(trajectory_duration); */
}

std::optional<RJ::Seconds> PlannerForRobot::get_time_left() const {
    // TODO(p-nayak): why does this say 3s even when the robot is on its point?
    // get the Traj out of the relevant [Trajectory, priority] tuple in
    // robot_trajectories_

    /*
    // TODO (PR #1970): fix TrajectoryCollection
    const auto& [latest_traj, priority] = robot_trajectories_->get(robot_id_);
    if (!latest_traj) {
        return std::nullopt;
    }
    return latest_traj->end_time() - RJ::now();
    */
    return std::nullopt;
}

}  // namespace planning
