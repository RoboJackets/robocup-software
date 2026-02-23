#include "rj_planning/planner_node.hpp"

namespace planning {

using RobotMove = rj_msgs::action::RobotMove;
using GoalHandleRobotMove = rclcpp_action::ServerGoalHandle<RobotMove>;

PlannerNode::PlannerNode()
    : rclcpp::Node("planner", rclcpp::NodeOptions{}
                                  .automatically_declare_parameters_from_overrides(true)
                                  .allow_undeclared_parameters(true)),
      global_state_(this) {
    planning_config_ = load_planning_config();

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
    robot_trajectories_ = std::make_shared<TrajectoryCollection>();
    robot_planners_.reserve(kNumShells);
    for (size_t i = 0; i < kNumShells; i++) {
        auto planner = std::make_unique<PlannerForRobot>(i, this, robot_trajectories_,
                                                         global_state_, &planning_config_);
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

PlanningConfig PlannerNode::load_planning_config() {
    PlanningConfig config;
    this->get_parameter_or("timeout", config.timeout, config.timeout);

    this->get_parameter_or("constraints.max_translational_speed",
                           config.constraints.max_translational_speed,
                           config.constraints.max_translational_speed);
    this->get_parameter_or("constraints.max_translational_accel",
                           config.constraints.max_translational_accel,
                           config.constraints.max_translational_accel);
    this->get_parameter_or("constraints.max_rotational_speed",
                           config.constraints.max_rotational_speed,
                           config.constraints.max_rotational_speed);
    this->get_parameter_or("constraints.max_rotational_accel",
                           config.constraints.max_rotational_accel,
                           config.constraints.max_rotational_accel);

    this->get_parameter_or("replanner.pos_change_threshold", config.replanner.pos_change_threshold,
                           config.replanner.pos_change_threshold);
    this->get_parameter_or("replanner.vel_change_threshold", config.replanner.vel_change_threshold,
                           config.replanner.vel_change_threshold);
    this->get_parameter_or("replanner.partial_replan_lead_time",
                           config.replanner.partial_replan_lead_time,
                           config.replanner.partial_replan_lead_time);
    this->get_parameter_or("replanner.off_path_threshold", config.replanner.off_path_threshold,
                           config.replanner.off_path_threshold);

    this->get_parameter_or("rrt.enable_debug_drawing", config.rrt.enable_debug_drawing,
                           config.rrt.enable_debug_drawing);
    this->get_parameter_or("rrt.step_size", config.rrt.step_size, config.rrt.step_size);
    this->get_parameter_or("rrt.goal_bias", config.rrt.goal_bias, config.rrt.goal_bias);
    this->get_parameter_or("rrt.waypoint_bias", config.rrt.waypoint_bias, config.rrt.waypoint_bias);
    this->get_parameter_or("rrt.min_iterations", config.rrt.min_iterations,
                           config.rrt.min_iterations);
    this->get_parameter_or("rrt.max_iterations", config.rrt.max_iterations,
                           config.rrt.max_iterations);

    this->get_parameter_or("intermediate.min_scale", config.intermediate.min_scale,
                           config.intermediate.min_scale);
    this->get_parameter_or("intermediate.max_scale", config.intermediate.max_scale,
                           config.intermediate.max_scale);
    this->get_parameter_or("intermediate.min_angle", config.intermediate.min_angle,
                           config.intermediate.min_angle);
    this->get_parameter_or("intermediate.max_angle", config.intermediate.max_angle,
                           config.intermediate.max_angle);
    this->get_parameter_or("intermediate.num_intermediates", config.intermediate.num_intermediates,
                           config.intermediate.num_intermediates);
    this->get_parameter_or("intermediate.step_size", config.intermediate.step_size,
                           config.intermediate.step_size);

    this->get_parameter_or("escape.step_size", config.escape.step_size, config.escape.step_size);
    this->get_parameter_or("escape.goal_change_threshold", config.escape.goal_change_threshold,
                           config.escape.goal_change_threshold);

    this->get_parameter_or("pivot.radius_multiplier", config.pivot.radius_multiplier,
                           config.pivot.radius_multiplier);

    this->get_parameter_or("collect.ball_speed_approach_direction_cutoff",
                           config.collect.ball_speed_approach_direction_cutoff,
                           config.collect.ball_speed_approach_direction_cutoff);
    this->get_parameter_or("collect.approach_accel_scale", config.collect.approach_accel_scale,
                           config.collect.approach_accel_scale);
    this->get_parameter_or("collect.control_accel_scale", config.collect.control_accel_scale,
                           config.collect.control_accel_scale);
    this->get_parameter_or("collect.approach_dist_target", config.collect.approach_dist_target,
                           config.collect.approach_dist_target);
    this->get_parameter_or("collect.touch_delta_speed", config.collect.touch_delta_speed,
                           config.collect.touch_delta_speed);
    this->get_parameter_or("collect.velocity_control_scale", config.collect.velocity_control_scale,
                           config.collect.velocity_control_scale);
    this->get_parameter_or("collect.dist_cutoff_to_control", config.collect.dist_cutoff_to_control,
                           config.collect.dist_cutoff_to_control);
    this->get_parameter_or("collect.dist_cutoff_to_approach",
                           config.collect.dist_cutoff_to_approach,
                           config.collect.dist_cutoff_to_approach);
    this->get_parameter_or("collect.vel_cutoff_to_control", config.collect.vel_cutoff_to_control,
                           config.collect.vel_cutoff_to_control);
    this->get_parameter_or("collect.stop_dist_scale", config.collect.stop_dist_scale,
                           config.collect.stop_dist_scale);
    this->get_parameter_or("collect.target_point_lowpass_gain",
                           config.collect.target_point_lowpass_gain,
                           config.collect.target_point_lowpass_gain);

    this->get_parameter_or("settle.ball_speed_percent_for_dampen",
                           config.settle.ball_speed_percent_for_dampen,
                           config.settle.ball_speed_percent_for_dampen);
    this->get_parameter_or("settle.search_start_dist", config.settle.search_start_dist,
                           config.settle.search_start_dist);
    this->get_parameter_or("settle.search_end_dist", config.settle.search_end_dist,
                           config.settle.search_end_dist);
    this->get_parameter_or("settle.search_inc_dist", config.settle.search_inc_dist,
                           config.settle.search_inc_dist);
    this->get_parameter_or("settle.intercept_buffer_time", config.settle.intercept_buffer_time,
                           config.settle.intercept_buffer_time);
    this->get_parameter_or("settle.target_point_gain", config.settle.target_point_gain,
                           config.settle.target_point_gain);
    this->get_parameter_or("settle.ball_vel_gain", config.settle.ball_vel_gain,
                           config.settle.ball_vel_gain);
    this->get_parameter_or("settle.shortcut_dist", config.settle.shortcut_dist,
                           config.settle.shortcut_dist);
    this->get_parameter_or("settle.max_ball_angle_for_reset",
                           config.settle.max_ball_angle_for_reset,
                           config.settle.max_ball_angle_for_reset);
    this->get_parameter_or("settle.max_ball_vel_for_path_reset",
                           config.settle.max_ball_vel_for_path_reset,
                           config.settle.max_ball_vel_for_path_reset);
    this->get_parameter_or("settle.max_bounce_angle", config.settle.max_bounce_angle,
                           config.settle.max_bounce_angle);

    return config;
}

}  // namespace planning
