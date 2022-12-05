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

    // TODO(NATE): Update QoS for service to fit correct number
    robot_communication_srv_ = create_service<rj_msgs::srv::AgentCommunication>(
        fmt::format("agent_{}_incoming", r_id),
        [this](const std::shared_ptr<rj_msgs::srv::AgentCommunication::Request> request,
                std::shared_ptr<rj_msgs::srv::AgentCommunication::Response> response) {
                    receive_communication_callback(request, response); });

    // Create clients
    for (size_t i = 0; i < kNumShells; i++) {
        robot_communication_cli_[i] = create_client<rj_msgs::srv::AgentCommunication>(fmt::format("agent_{}_incoming", i));
    }

    // TODO(Kevin): make ROS param for this
    int hz = 10;
    get_task_timer_ = create_wall_timer(std::chrono::milliseconds(1000 / hz),
                                        std::bind(&AgentActionClient::get_task, this));

    int agent_communication_hz = 60;
    get_communication_timer_ = create_wall_timer(std::chrono::milliseconds(1000 / agent_communication_hz), [this](){ get_communication(); });
}

void AgentActionClient::world_state_callback(const rj_msgs::msg::WorldState::SharedPtr& msg) {
    WorldState world_state = rj_convert::convert_from_ros(*msg);
    auto lock = std::lock_guard(world_state_mutex_);
    last_world_state_ = std::move(world_state);

    if (current_position_ == nullptr) {
        return;
    }

    current_position_->update_world_state(world_state);
}

void AgentActionClient::coach_state_callback(const rj_msgs::msg::CoachState::SharedPtr& msg) {
    if (current_position_ == nullptr) {
        return;
    }

    current_position_->update_coach_state(*msg);
}

void AgentActionClient::get_task() {
    // TODO: change this default to defense? or NOP?
    if (current_position_ == nullptr) {
        // TODO: change this once coach node merged
        if (robot_id_ == 0) {
            current_position_ = std::make_unique<Goalie>(robot_id_);
        } else if (robot_id_ == 1) {
            current_position_ = std::make_unique<Defense>(robot_id_);
        } else {
            current_position_ = std::make_unique<Offense>(robot_id_);
        }
    }

    auto task = current_position_->get_task();
    if (task != last_task_) {
        last_task_ = task;
        send_new_goal();
    }
}

void AgentActionClient::send_new_goal() {
    using namespace std::placeholders;

    if (!client_ptr_->wait_for_action_server()) {
        SPDLOG_ERROR("Action server not available after waiting");
        rclcpp::shutdown();
    }

    auto goal_msg = RobotMove::Goal();
    goal_msg.robot_intent = last_task_;

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
        current_position_->goal_canceled_ = true;
    }
}

void AgentActionClient::feedback_callback(
    GoalHandleRobotMove::SharedPtr, const std::shared_ptr<const RobotMove::Feedback> feedback) {
    double time_left = rj_convert::convert_from_ros(feedback->time_left).count();
    current_position_->time_left_ = time_left;
}

void AgentActionClient::result_callback(const GoalHandleRobotMove::WrappedResult& result) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            // TODO: handle other return codes
            current_position_->is_done_ = true;
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

void AgentActionClient::get_communication() {
    if (current_position_ == nullptr) {
        if (robot_id_ == 0) {
            current_position_ = std::make_unique<Goalie>(robot_id_);
        } else if (robot_id_ == 1) {
            current_position_ = std::make_unique<Defense>(robot_id_);
        } else {
            current_position_ = std::make_unique<Offense>(robot_id_);
        }
    }

    auto communication_request = current_position_->send_communication_request();
    rj_msgs::msg::AgentRequest request = communication_request.request;

    bool robots_visible = false;
    for (u_int8_t i = 0; i < kNumShells; i++) {
        if (this->world_state()->get_robot(true, i).visible) {
            robots_visible = true;
            break;
        }
    }

    if (request != last_communication_ && robots_visible) {
        last_communication_ = request;
        if (communication_request.broadcast) {
            send_broadcast(request);
        } else if (communication_request.num_targets > 1) {
            std::vector<u_int8_t> targets(std::begin(communication_request.target_agents), std::end(communication_request.target_agents));
            send_multicast(targets, request);
        } else if (communication_request.num_targets == 1) {
            send_unicast(communication_request.target_agents[0], request);
        } else {
            SPDLOG_WARN("BAD REQUEST NOT SENDING ANY COMMUNICATION");
        }
    }
}

void AgentActionClient::send_unicast(int robot_id, rj_msgs::msg::AgentRequest agent_request) {
    auto request = std::make_shared<rj_msgs::srv::AgentCommunication::Request>();
    request->agent_request = agent_request;

    robot_communication_cli_[robot_id]->async_send_request(
        request,
        [this, robot_id](const std::shared_future<rj_msgs::srv::AgentCommunication::Response::SharedPtr> response) {
            receive_response_callback(response, robot_id);
        }
    );
}

void AgentActionClient::send_broadcast(rj_msgs::msg::AgentRequest agent_request) {
    auto request = std::make_shared<rj_msgs::srv::AgentCommunication::Request>();
    request->agent_request = agent_request;

    for (size_t i = 0; i < kNumShells; i++) {
        if (i != robot_id_ && this->world_state()->get_robot(true, i).visible) {
            SPDLOG_INFO("\033[92mSENDING REQUEST TO AGENT: {}\033[0m", i);
            robot_communication_cli_[i]->async_send_request(
                request,
                [this, i](const std::shared_future<rj_msgs::srv::AgentCommunication::Response::SharedPtr> response) {
                    receive_response_callback(response, i);
                }
            );
        }
    }
}

void AgentActionClient::send_multicast(std::vector<u_int8_t> robot_ids, rj_msgs::msg::AgentRequest agent_request) {
    auto request = std::make_shared<rj_msgs::srv::AgentCommunication::Request>();
    request->agent_request = agent_request;

    for (int i : robot_ids) {
        robot_communication_cli_[i]->async_send_request(
            request,
            [this, i](const std::shared_future<rj_msgs::srv::AgentCommunication::Response::SharedPtr> response) {
                receive_response_callback(response, i);
            }
        );
    }
}

void AgentActionClient::receive_communication_callback(const std::shared_ptr<rj_msgs::srv::AgentCommunication::Request>& request,
                                                        std::shared_ptr<rj_msgs::srv::AgentCommunication::Response>& response) {
    // TODO: change this default to defense? or NOP?
    if (current_position_ == nullptr) {
        rj_msgs::msg::AgentResponse agent_response{};
        rj_msgs::msg::Acknowledge acknowledgement{};
        acknowledgement.acknowledged = true;
        agent_response.acknowledge_response = {acknowledgement};
        response->agent_response = agent_response;
    } else {
        // Convert agent request into AgentToPosCommRequest
        rj_msgs::msg::AgentToPosCommRequest agent_request{};
        agent_request.request = request->agent_request;

        // Give the current position the request and receive the response to send back
        rj_msgs::msg::PosToAgentCommResponse pos_to_agent_response = current_position_->receive_communication_request(agent_request);

        // Convert PosToAgentCommResponse into AgentResponse
        rj_msgs::msg::AgentResponse agent_response = pos_to_agent_response.response;

        // Send the agent's response back to the client who asked
        response->agent_response = agent_response;
    }
}

void AgentActionClient::receive_response_callback(const std::shared_future<rj_msgs::srv::AgentCommunication::Response::SharedPtr>& response, int robot_id) {
    // TODO: change this default to defense? or NOP?
    if (current_position_ == nullptr) {
        // TODO: change this once coach node merged
        if (robot_id_ == 0) {
            current_position_ = std::make_unique<Goalie>(robot_id_);
        } else if (robot_id_ == 1) {
            current_position_ = std::make_unique<Defense>(robot_id_);
        } else {
            current_position_ = std::make_unique<Offense>(robot_id_);
        }
    }

    rj_msgs::msg::AgentToPosCommResponse agent_to_position_response{};
    agent_to_position_response.robot_id = robot_id;
    agent_to_position_response.response = response.get()->agent_response;

    // Relay information to the position
    current_position_->receive_communication_response(agent_to_position_response);
}

[[nodiscard]] WorldState* AgentActionClient::world_state() {
    // thread-safe getter for world_state (see update_world_state())
    auto lock =std::lock_guard(world_state_mutex_);
    return &last_world_state_;
}

}  // namespace strategy
