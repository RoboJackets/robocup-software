#include "rj_planning/planner_node.hpp"

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

    // Replanner params
    this->replanner_off_path_threshold =
        this->get_parameter_or("replanner.off_path_threshold", 0.1);
    this->replanner_partial_replan_lead_time =
        this->get_parameter_or("replanner.partial_replan_lead_time", 0.1);
    this->replanner_pos_change_threshold =
        this->get_parameter_or("replanner.pos_change_threshold", 0.025);
    this->replanner_vel_change_threshold =
        this->get_parameter_or("replanner.vel_change_threshold", 0.025);

    this->constraints_params_ = build_constraints_params();
    this->rrt_params_ = build_rrt_params();
    this->line_test_params_ = build_line_test_params();
    this->pivot_radius_multiplier_param_ = get_parameter_or("pivot.radius_multiplier", 1.0);
    this->timeout_param_ = get_parameter_or("timeout", 0.1);
    this->use_sim_time_param_ = get_parameter_or("use_sim_time", false);

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
        auto planner =
            std::make_unique<PlannerForRobot>(i, this, robot_trajectories_, global_state_);
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

ConstraintsParams PlannerNode::build_constraints_params() {
    ConstraintsParams p;
    p.max_rotational_accel =
        this->get_parameter_or("constraints.max_rotational_accel", 3.0);
    p.max_rotational_speed =
        this->get_parameter_or("constraints.max_rotational_speed", 3.0);
    p.max_translational_accel =
        this->get_parameter_or("constraints.max_translational_accel", 2.0);
    p.max_translational_speed =
        this->get_parameter_or("constraints.max_translational_speed", 2.0);
    return p;
}

RrtParams PlannerNode::build_rrt_params() {
    RrtParams p;
    p.enable_debug_drawing =
        this->get_parameter_or("rrt.enable_debug_drawing", false);
    p.goal_bias =
        this->get_parameter_or("rrt.goal_bias", 0.3);
    p.step_size =
        this->get_parameter_or("rrt.step_size", 0.15);
    p.waypoint_bias =
        this->get_parameter_or("rrt.waypoint_bias", 0.5);
    p.max_iterations =
        this->get_parameter_or("rrt.max_iterations", 250);
    p.min_iterations =
        this->get_parameter_or("rrt.min_iterations", 0);
    return p;
}

LineTestParams PlannerNode::build_line_test_params() {
    LineTestParams p;
    p.start_x = this->get_parameter_or("line_test.start_x", 7.0);
    p.start_y = this->get_parameter_or("line_test.start_y", -1.0);
    p.end_x = this->get_parameter_or("line_test.end_x", 7.0);
    p.end_y = this->get_parameter_or("line_test.end_y", 1.0);
    p.robot_id = this->get_parameter_or("line_test.robot_id", 1);
    return p;
}

}  // namespace planning
