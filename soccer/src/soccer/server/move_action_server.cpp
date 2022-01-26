#include "move_action_server.hpp"

namespace server {
using Move = rj_msgs::action::Move;
using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;
MoveActionServer ::MoveActionServer(const rclcpp::NodeOptions& options)
    : Node("move_action_server", options) {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Move>(
        this->get_node_base_interface(),
        this->get_node_clock_interface(),
        this->get_node_logging_interface(),
        this->get_node_waitables_interface(),
        "move", std::bind(&MoveActionServer::handle_goal, this, _1, _2),
        std::bind(&MoveActionServer::handle_cancel, this, _1),
        std::bind(&MoveActionServer::handle_accepted, this, _1));

    world_state_sub_ = this->create_subscription<WorldState::Msg>(
        vision_filter::topics::kWorldStatePub, rclcpp::QoS(1),
        [this](WorldState::Msg::SharedPtr world_state_msg) {  // NOLINT
            this->robot_states_ = rj_convert::convert_from_ros(world_state_msg->our_robots);

        });

    this->intent_pubs_.reserve(kNumShells);
    this->trajectory_subs_.reserve(kNumShells);
    this->robot_trajectories_.reserve(kNumShells);
    this->test_desired_states_.reserve(kNumShells);
    this->test_desired_states_.assign(kNumShells, false);
    this->test_accept_goal_.reserve(kNumShells);
    this->test_accept_goal_.assign(kNumShells, true);
    this->target_positions.fill((-1, -1));

    for (size_t i = 0; i < kNumShells; i++) {
        intent_pubs_.emplace_back(this->create_publisher<RobotIntent>(
            action_server::topics::robot_intent_pub(i), rclcpp::QoS(1).transient_local()));

        this->create_subscription<planning::Trajectory::Msg>(
            planning::topics::trajectory_pub(i), rclcpp::QoS(1),
            [this, i](planning::Trajectory::Msg::SharedPtr trajectory) {  // NOLINT
                trajectory_ = rj_convert::convert_from_ros(*trajectory);
                this->robot_trajectories_[i] = (trajectory_);
            });

        this->create_subscription<RobotState::Msg>(
            control::topics::desired_state_pub(i), rclcpp::QoS(1),
            [this, i](RobotState::Msg::SharedPtr desired_state) {  // NOLINT
                this->test_desired_states_[i] = true;
                // TODO : add mutex here?
                this->robot_desired_states_[i] = (rj_convert::convert_from_ros(*desired_state));
            });
    }
}

rclcpp_action::GoalResponse MoveActionServer ::handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                           std::shared_ptr<const Move::Goal> goal) {
    // TODO: only accept if goal is move action
    //  (so just check that the empty command is not filled)
    // std::cout << "handle goal reached" << std::endl;
    (void)uuid;

    int robot_id = goal->server_intent.robot_id;
    RobotIntent robot_intent = goal->server_intent.intent;
    const planning::MotionCommand motion_command =
        rj_convert::convert_from_ros(robot_intent.motion_command);
    if (robot_states_[robot_id].visible && std::holds_alternative<planning::PathTargetCommand>(motion_command)) {
        const auto target_position = rj_convert::convert_from_ros(
            goal->server_intent.intent.motion_command.path_target_command[0].target.position);
        rj_geometry::Point old_position = target_positions[robot_id];
        if (target_position == old_position) {
            target_positions[robot_id] = target_position;
            return rclcpp_action::GoalResponse::REJECT;
        } else {
            target_positions[robot_id] = target_position;
        }
    }

    //accept_mutexes[robot_id].lock();
    // TODO : remove this once fixed slowness
    // if (this->test_accept_goal_[robot_id]) {
        //this->test_accept_goal_[robot_id] = false;
        //accept_mutexes[robot_id].unlock();
        //return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    //}
    //accept_mutexes[robot_id].unlock();
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveActionServer ::handle_cancel(
    const std::shared_ptr<GoalHandleMove> goal_handle) {
    std::cout << "cancel reached" << std::endl;
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveActionServer ::handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle) {
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    using namespace std::placeholders;
    std::cout << "accepted reached" << std::endl;
    std::thread{std::bind(&MoveActionServer::execute, this, _1), goal_handle}.detach();
}

void MoveActionServer ::execute(const std::shared_ptr<GoalHandleMove> goal_handle) {
    std::shared_ptr<const Move::Goal> goal = goal_handle->get_goal();
    rj_msgs::msg::ServerIntent server_intent = goal->server_intent;
    rj_msgs::msg::RobotIntent robot_intent = server_intent.intent;

    const planning::MotionCommand motion_command =
        rj_convert::convert_from_ros(robot_intent.motion_command);
    int robot_id = server_intent.robot_id;
    std::cout << robot_id << std::endl;

    RJ::Time base_time = RJ::Time();
    bool tested = this->test_desired_states_.at(robot_id);
    RJ::Time old_timestamp = tested ? robot_desired_states_[robot_id].timestamp : base_time;

    std::shared_ptr<Move::Result> result = std::make_shared<Move::Result>();

    // TODO : remove if statement once move action server is only responsible for move actions
    rclcpp::Rate loop_rate(1);
    if (!std::holds_alternative<planning::EmptyCommand>(motion_command)) {
        do {
            this->intent_pubs_[robot_id]->publish(robot_intent);
            if (goal_handle->is_canceling()) {
                result->is_done = true;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal Canceled");
                return;
            }

            std::shared_ptr<Move::Feedback> feedback = std::make_shared<Move::Feedback>();
            planning::Trajectory robot_trajectory = this->robot_trajectories_[robot_id];

            if (!robot_trajectory.empty()) {
                planning::Trajectory::Msg trajectory_msg =
                    rj_convert::convert_to_ros(robot_trajectory);
                feedback->trajectory = trajectory_msg;
            }

            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "published feedback");
            loop_rate.sleep();
        } while (test_desired_states_[robot_id] && robot_desired_states_[robot_id].visible &&
                 robot_desired_states_[robot_id].timestamp < old_timestamp);
    }
    //accept_mutexes[robot_id].lock();
    //this->test_accept_goal_[robot_id] = true;
    //accept_mutexes[robot_id].unlock()
    if (goal_handle->is_canceling()) {
        result->is_done = true;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal Canceled");
        return;
    }

    if (rclcpp::ok()) {
        result->is_done = true;
        goal_handle->succeed(result);
    }
}
}  // namespace server
