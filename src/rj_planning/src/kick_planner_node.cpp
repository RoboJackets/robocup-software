#include "rj_planning/kick_planner_node.hpp"

namespace planning {

using RobotMove = rj_msgs::action::RobotMove;
using GoalHandleRobotMove = rclcpp_action::ServerGoalHandle<RobotMove>;

KickPlannerNode::KickPlannerNode()
    : rclcpp::Node("planner", rclcpp::NodeOptions{}
                                  .automatically_declare_parameters_from_overrides(true)
                                  .allow_undeclared_parameters(true)),
      global_state_(this),
      param_provider_{this, kPlanningParamModule} {
    // for _1, _2 etc. below
    using namespace std::placeholders;

    // set up ActionServer + callbacks

    this->manipulator_server_ = rclcpp_action::create_server<RobotMove>(
        this->get_node_base_interface(), this->get_node_clock_interface(),
        this->get_node_logging_interface(), this->get_node_waitables_interface(),
        "robot_manipulator", std::bind(&KickPlannerNode::handle_goal, this, _1, _2),
        std::bind(&KickPlannerNode::handle_cancel, this, _1),
        std::bind(&KickPlannerNode::handle_accepted, this, _1));

    for (int i = 0; i < 6; i++) {
        manipulator_pubs_.emplace_back(create_publisher<rj_msgs::msg::ManipulatorSetpoint>(
            control::topics::manipulator_setpoint_topic(i), rclcpp::QoS(10)));
    }
}

rclcpp_action::GoalResponse KickPlannerNode::handle_goal(
    const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const RobotMove::Goal> goal) {
    (void)uuid;
    auto delay = std::chrono::milliseconds(1000 / 60);
    rclcpp::Rate loop_rate(delay);

    // TODO(p-nayak): REJECT duplicate goal requests so we aren't constantly replanning them

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

rclcpp_action::CancelResponse KickPlannerNode::handle_cancel(
    const std::shared_ptr<GoalHandleRobotMove> goal_handle) {
    (void)goal_handle;
    std::shared_ptr<const RobotMove::Goal> goal = goal_handle->get_goal();
    return rclcpp_action::CancelResponse::ACCEPT;
}

void KickPlannerNode::handle_accepted(const std::shared_ptr<GoalHandleRobotMove> goal_handle) {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    // execute() will block (loop) until completion (either success or canceled by client)
    using namespace std::placeholders;
    std::thread{std::bind(&KickPlannerNode::execute, this, _1), goal_handle}.detach();
}

void KickPlannerNode::execute(const std::shared_ptr<GoalHandleRobotMove> goal_handle) {
    // create ptrs to Goal, Result objects per ActionServer API
    std::shared_ptr<const RobotMove::Goal> goal = goal_handle->get_goal();
    std::shared_ptr<RobotMove::Result> result = std::make_shared<RobotMove::Result>();

    // get correct PlannerForRobot object for this robot_id
    int robot_id = goal->robot_intent.robot_id;
    // reference to unique_ptr to avoid transferring ownership

    auto& robot_task = server_task_states_.at(robot_id);

    double dribbler_speed;
    planning::Trajectory::TriggerMode trigger_mode;
    planning::Trajectory::ShootMode shoot_mode;

    if (goal->robot_intent.dribbler_mode != RobotIntent::DribblerMode::DEFAULT) {
        dribbler_speed =
            (goal->robot_intent.dribbler_mode == RobotIntent::DribblerMode::ON) ? 255.0 : 0.0;
    }

    switch (goal->robot_intent.trigger_mode) {
        case RobotIntent::TriggerMode::STAND_DOWN:
            trigger_mode = planning::Trajectory::TriggerMode::STAND_DOWN;
            break;
        case RobotIntent::TriggerMode::IMMEDIATE:
            trigger_mode = planning::Trajectory::TriggerMode::IMMEDIATE;
            break;
        case RobotIntent::TriggerMode::ON_BREAK_BEAM:
            trigger_mode = planning::Trajectory::TriggerMode::ON_BREAK_BEAM;
            break;
        default:
            break;
    }

    switch (goal->robot_intent.shoot_mode) {
        case RobotIntent::ShootMode::CHIP:
            shoot_mode = planning::Trajectory::ShootMode::CHIP;
            break;
        case RobotIntent::ShootMode::KICK:
            shoot_mode = planning::Trajectory::ShootMode::KICK;
            break;
    }

    // send the kick/dribble commands to the radio
    manipulator_pubs_.at(robot_id)->publish(rj_msgs::build<rj_msgs::msg::ManipulatorSetpoint>()
                                                .shoot_mode(shoot_mode)
                                                .trigger_mode(trigger_mode)
                                                .kick_speed(goal->robot_intent.kick_speed)
                                                .dribbler_speed(dribbler_speed));
    result->is_done = true;
    goal_handle->succeed(result);

    robot_task.is_executing = false;
}

}  // namespace planning