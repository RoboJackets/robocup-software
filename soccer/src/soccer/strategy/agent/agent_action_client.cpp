#include "agent_action_client.hpp"

namespace strategy {
using RobotMove = rj_msgs::action::RobotMove;
using GoalHandleRobotMove = rclcpp_action::ClientGoalHandle<RobotMove>;

AgentActionClient::AgentActionClient() : AgentActionClient(0) {
    // unclear why I need to explicitly create a default constructor, but compiler throws error when
    // not here https://stackoverflow.com/questions/47704900/error-use-of-deleted-function
}

AgentActionClient::AgentActionClient(int r_id)
    : robot_id_(r_id),
      rclcpp::Node(fmt::format("agent_{}_action_client_node", r_id),
                   rclcpp::NodeOptions{}
                       .automatically_declare_parameters_from_overrides(true)
                       .allow_undeclared_parameters(true)) {
    SPDLOG_INFO("my robot_id_ {}", robot_id_);

    // create a ptr to ActionClient
    client_ptr_ = rclcpp_action::create_client<RobotMove>(this, "robot_move");

    world_state_sub_ = create_subscription<rj_msgs::msg::WorldState>(
        "vision_filter/world_state", 1,
        [this](rj_msgs::msg::WorldState::SharedPtr msg) { world_state_callback(msg); });

    coach_state_sub_ = create_subscription<rj_msgs::msg::CoachState>(
        "strategy/coach_state", 1,
        [this](rj_msgs::msg::CoachState::SharedPtr msg) { coach_state_callback(msg); });

    positions_sub_ = create_subscription<rj_msgs::msg::Position>(
        "strategy/positions", 1,
        [this](rj_msgs::msg::Position::SharedPtr msg) { update_position(msg); });

    // TODO(Kevin): make ROS param for this
    int hz = 10;
    get_task_timer_ = create_wall_timer(std::chrono::milliseconds(1000 / hz),
                                        std::bind(&AgentActionClient::get_task, this));
}

void AgentActionClient::world_state_callback(const rj_msgs::msg::WorldState::SharedPtr& msg) {
    if (current_position_ == nullptr) {
        return;
    }

    WorldState world_state = rj_convert::convert_from_ros(*msg);
    current_position_->update_world_state(world_state);
}

void AgentActionClient::coach_state_callback(const rj_msgs::msg::CoachState::SharedPtr& msg) {
    if (current_position_ == nullptr) {
        return;
    }

    current_position_->update_coach_state(*msg);
}

void AgentActionClient::get_task() {
    SPDLOG_INFO("Getting task for robot {}", robot_id_);
    if (current_position_ == nullptr) {
        if (robot_id_ == 0) {
            current_position_ = std::make_unique<Goalie>(robot_id_);
        } else if (robot_id_ == 1) {
            current_position_ = std::make_unique<Defense>(robot_id_);
        } else {
            current_position_ = std::make_unique<Offense>(robot_id_);
        }
    }

    auto optional_task = current_position_->get_task();
    if (optional_task.has_value()) {
        RobotIntent task = optional_task.value();

        // note that because these are our RobotIntent structs, this comparison
        // uses our custom struct overloads
        if (task != last_task_) {
            SPDLOG_INFO("robot {} has new task '{}'", robot_id_, task.motion_command_name);
            last_task_ = task;
            send_new_goal();
        }
    }
}

void AgentActionClient::update_position(const rj_msgs::msg::Position::SharedPtr& msg) {
    // TODO remove this debug
    /* for (int i = 0; i < 6; i++) { */
    /*     SPDLOG_INFO("position at {}: {}", i, msg->client_positions.at(i)); */
    /* } */
    // SPDLOG_INFO("{}'s position : {}", robot_id_, msg->client_positions[robot_id_]);
    if (robot_id_ == 0) {
        return;
    }

    std::unique_ptr<Position> next_position_;
    switch (msg->client_positions[robot_id_]) {
        case 1:
            next_position_ = std::make_unique<Defense>(robot_id_);
            break;
        case 2:
            next_position_ = std::make_unique<Offense>(robot_id_);
            break;
    };

    if (current_position_ == nullptr ||
        next_position_->get_name() == current_position_->get_name()) {
        current_position_ = std::move(next_position_);
    }

    // TODO: send acknowledgement back to coach node (w/ robot ID)
}

void AgentActionClient::send_new_goal() {
    using namespace std::placeholders;

    if (!client_ptr_->wait_for_action_server()) {
        SPDLOG_ERROR("Action server not available after waiting");
        rclcpp::shutdown();
    }

    auto goal_msg = RobotMove::Goal();
    // must convert to ROS msg form in order to be sent across ROS nodes
    goal_msg.robot_intent = rj_convert::convert_to_ros(last_task_);

    auto send_goal_options = rclcpp_action::Client<RobotMove>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&AgentActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
        std::bind(&AgentActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback = std::bind(&AgentActionClient::result_callback, this, _1);
    client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

void AgentActionClient::goal_response_callback(
    std::shared_future<GoalHandleRobotMove::SharedPtr> future) {
    auto goal_handle = future.get();
    if (!goal_handle) {
        current_position_->set_goal_canceled();
    }
}

void AgentActionClient::feedback_callback(
    GoalHandleRobotMove::SharedPtr, const std::shared_ptr<const RobotMove::Feedback> feedback) {
    double time_left = rj_convert::convert_from_ros(feedback->time_left).count();
    current_position_->set_time_left(time_left);
}

void AgentActionClient::result_callback(const GoalHandleRobotMove::WrappedResult& result) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            // TODO: handle other return codes
            current_position_->set_is_done();
            break;
        case rclcpp_action::ResultCode::ABORTED:
            return;
        case rclcpp_action::ResultCode::CANCELED:
            return;
        default:
            return;
    }
    /* SPDLOG_INFO("Result received: {}", result.result->is_done); */
}

}  // namespace strategy
