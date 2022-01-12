#include "move_action_server.hpp"

namespace server {
using Move = rj_msgs::action::Move;
using GoalHandleMove = rclcpp_action::ServerGoalHandle<Move>;
MoveActionServer ::MoveActionServer(const rclcpp::NodeOptions& options)
    : Node("move_action_server", options) {
    using namespace std::placeholders;

    this->robot_test_.reserve(kNumShells);

    this->action_server_ = rclcpp_action::create_server<Move>(
        this, "move", std::bind(&MoveActionServer::handle_goal, this, _1, _2),
        std::bind(&MoveActionServer::handle_cancel, this, _1),
        std::bind(&MoveActionServer::handle_accepted, this, _1));

    intent_pubs_.reserve(kNumShells);
    world_state_sub_ = this->create_subscription<WorldState::Msg>(
        vision_filter::topics::kWorldStatePub, rclcpp::QoS(1),
        [this](WorldState::Msg::SharedPtr world_state_msg) {  // NOLINT
            this->robot_states_ = rj_convert::convert_from_ros(world_state_msg->our_robots);

        });

    this->trajectory_subs_.reserve(kNumShells);
    this->robot_trajectories_.reserve(kNumShells);
    for (int i = 0; i < kNumShells; i++) {
        intent_pubs_.emplace_back(this->create_publisher<RobotIntent>(
            action_server::topics::robot_intent_pub(i), rclcpp::QoS(1).transient_local()));
        // TODO: change this (causes crashing)
        this->create_subscription<planning::Trajectory::Msg>(
            planning::topics::trajectory_pub(i), rclcpp::QoS(1),
            [this, i](planning::Trajectory::Msg::SharedPtr trajectory) {  // NOLINT
                int id = i;
                trajectory_ = rj_convert::convert_from_ros(*trajectory);
                this->robot_trajectories_[id] = trajectory_;
            });
        this->create_subscription<RobotState::Msg>(
            control::topics::desired_state_pub(i), rclcpp::QoS(1),
            [this, i](RobotState::Msg::SharedPtr desired_state) {  // NOLINT
                int id = i;
                this->robot_desired_states_[id] = rj_convert::convert_from_ros(*desired_state);
            });
    }
}

rclcpp_action::GoalResponse MoveActionServer ::handle_goal(const rclcpp_action::GoalUUID& uuid,
                                                           std::shared_ptr<const Move::Goal> goal) {
    std::cout << "handle goal reached" << std::endl;

    // rj_convert::convert_from_ros(
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MoveActionServer ::handle_cancel(
    const std::shared_ptr<GoalHandleMove> goal_handle) {
    // std::cout << "cancel reached" << std::endl;
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void MoveActionServer ::handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle) {
    using namespace std::placeholders;
    // std::cout << "accepted reached" << std::endl;
    std::thread{std::bind(&MoveActionServer::execute, this, _1), goal_handle}.detach();
}

void MoveActionServer ::execute(const std::shared_ptr<GoalHandleMove> goal_handle) {
    std::cout << "executing" << std::endl;
    const auto goal = goal_handle->get_goal();
    rj_msgs::msg::ServerIntent server_intent = goal->server_intent;
    volatile bool is_move =  server_intent.is_move;
    rj_msgs::msg::RobotIntent robot_intent = server_intent.intent;
    int robot_id = server_intent.robot_id;
    std::cout << robot_id << std::endl;
    this->intent_pubs_[robot_id]->publish(robot_intent);
    auto result = std::make_shared<Move::Result>();

    // TODO: get feedback from planner node (and fix below)
    //  (not working use motion control target_state instead for while cond.)
    auto old_timestamp = robot_desired_states_[robot_id].timestamp;

    if (is_move) {
        const auto target_position = rj_convert::convert_from_ros(
            goal->server_intent.intent.motion_command.path_target_command[0].target.position);
        do {
            if (goal_handle->is_canceling()) {
                result->is_done = true;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal Canceled");
                return;
            }
            auto feedback = std::make_shared<Move::Feedback>();
            planning::Trajectory robot_trajectory = this->robot_trajectories_[robot_id];
            if (!robot_trajectory.empty()) {
                planning::Trajectory::Msg trajectory_msg =
                    rj_convert::convert_to_ros(robot_trajectory);
                feedback->trajectory = trajectory_msg;
            }
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "published feedback");
        } while (robot_test_[robot_id] && robot_desired_states_[robot_id].visible
                 && robot_desired_states_[robot_id].timestamp <= old_timestamp);

    }

    // target_position != robot_desired_states_[robot_id].pose.position());

    // TODO : result should be set only when it is done
    result->is_done = true;
    robot_test_[robot_id] = true;
    goal_handle->succeed(result);
}
}  // namespace server

// RCLCPP_COMPONENTS_REGISTER_NODE(rj_robocup::MoveActionServer)
