#include "rj_strategy/agent/agent_action_client.hpp"

namespace strategy {
using RobotMove = rj_msgs::action::RobotMove;
using GoalHandleRobotMove = rclcpp_action::ClientGoalHandle<RobotMove>;

AgentActionClient::AgentActionClient(int r_id) : robot_id_{r_id} {
    node_ =
        std::make_shared<rclcpp::Node>(::fmt::format("agent_{}_action_client_node", r_id),
                                       rclcpp::NodeOptions{}
                                           .automatically_declare_parameters_from_overrides(true)
                                           .allow_undeclared_parameters(true));

    // create a ptr to ActionClient
    client_ptr_ = rclcpp_action::create_client<RobotMove>(node_, "robot_move");

    current_position_ = std::make_unique<RobotFactoryPosition>(r_id, node_);

    current_state_publisher_ = node_->create_publisher<AgentStateMsg>(
        fmt::format("strategy/positon/robot_state/robot_{}", r_id), 1);

    world_state_sub_ = node_->create_subscription<rj_msgs::msg::WorldState>(
        ::vision_filter::topics::kWorldStateTopic, 1,
        [this](rj_msgs::msg::WorldState::SharedPtr msg) { world_state_callback(msg); });

    play_state_sub_ = node_->create_subscription<rj_msgs::msg::PlayState>(
        ::referee::topics::kPlayStateTopic, 1,
        [this](const rj_msgs::msg::PlayState::SharedPtr msg) { play_state_callback(msg); });

    field_dimensions_sub_ = node_->create_subscription<rj_msgs::msg::FieldDimensions>(
        "config/field_dimensions", rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::FieldDimensions::SharedPtr msg) { field_dimensions_callback(msg); });

    alive_robots_sub_ = node_->create_subscription<rj_msgs::msg::AliveRobots>(
        ::radio::topics::kAliveRobotsTopic, 1,
        [this](rj_msgs::msg::AliveRobots::SharedPtr msg) { alive_robots_callback(msg); });

    game_settings_sub_ = node_->create_subscription<rj_msgs::msg::GameSettings>(
        "config/game_settings", 1,
        [this](rj_msgs::msg::GameSettings::SharedPtr msg) { game_settings_callback(msg); });

    goalie_id_sub_ = node_->create_subscription<rj_msgs::msg::Goalie>(
        ::referee::topics::kGoalieTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::Goalie::SharedPtr msg) { goalie_id_callback(msg->goalie_id); });

    override_play_sub_ = node_->create_subscription<rj_msgs::msg::OverridePosition>(
        "override_position/robot_" + std::to_string(r_id), 1,
        [this](rj_msgs::msg::OverridePosition::SharedPtr msg) {
            test_play_callback(msg);
        });  // NOLINT

    robot_communication_srv_ = node_->create_service<rj_msgs::srv::AgentCommunication>(
        fmt::format("agent_{}_incoming", r_id),
        [this](const std::shared_ptr<rj_msgs::srv::AgentCommunication::Request> request,
               std::shared_ptr<rj_msgs::srv::AgentCommunication::Response> response) {
            receive_communication_callback(request, response);
        });

    // Create clients
    for (size_t i = 0; i < kNumShells; i++) {
        robot_communication_cli_[i] = node_->create_client<rj_msgs::srv::AgentCommunication>(
            fmt::format("agent_{}_incoming", i));
    }

    int hz = 10;
    get_task_timer_ = node_->create_wall_timer(std::chrono::milliseconds(1000 / hz),
                                               std::bind(&AgentActionClient::get_task, this));

    int agent_communication_hz = 60;
    get_communication_timer_ = node_->create_wall_timer(
        std::chrono::milliseconds(1000 / agent_communication_hz), [this]() {
            get_communication();
            check_communication_timeout();
        });
}

rclcpp::Node::SharedPtr AgentActionClient::node() const { return node_; }

void AgentActionClient::world_state_callback(const rj_msgs::msg::WorldState::SharedPtr& msg) {
    if (current_position_ == nullptr) {
        return;
    }

    WorldState world_state = rj_convert::convert_from_ros(*msg);
    last_world_state_ = std::move(world_state);
}

void AgentActionClient::play_state_callback(const rj_msgs::msg::PlayState::SharedPtr& msg) {
    if (current_position_ == nullptr) {
        return;
    }

    PlayState play_state = rj_convert::convert_from_ros(*msg);
    play_state_ = play_state;
    current_position_->update_play_state(play_state);
}

void AgentActionClient::field_dimensions_callback(
    const rj_msgs::msg::FieldDimensions::SharedPtr& msg) {
    if (current_position_ == nullptr) {
        return;
    }

    FieldDimensions field_dimensions = rj_convert::convert_from_ros(*msg);
    field_dimensions_ = field_dimensions;
    current_position_->update_field_dimensions(field_dimensions);
}

void AgentActionClient::test_play_callback(
    const rj_msgs::msg::OverridePosition::SharedPtr& message) {
    if (current_position_) {
        current_position_->set_override_position(
            static_cast<strategy::OverridingPositions>(message->overriding_position));
    }
}

void AgentActionClient::goalie_id_callback(int goalie_id) {
    if (current_position_) {
        current_position_->set_goalie_id(goalie_id);
    }

    goalie_id_ = goalie_id;
}

void AgentActionClient::alive_robots_callback(const rj_msgs::msg::AliveRobots::SharedPtr& msg) {
    alive_robots_ = msg->alive_robots;

    current_position_->update_alive_robots(alive_robots_);
}

void AgentActionClient::game_settings_callback(const rj_msgs::msg::GameSettings::SharedPtr& msg) {
    is_simulated_ = msg->simulation;
}

bool AgentActionClient::check_robot_alive(u_int8_t robot_id) {
    if (!is_simulated_) {
        return alive_robots_.at(robot_id);
    }
    if (last_world_state_.get_robot(true, robot_id).visible) {
        rj_geometry::Point robot_position =
            last_world_state_.get_robot(true, robot_id).pose.position();
        rj_geometry::Rect padded_field_rect = field_dimensions_.field_coordinates();
        padded_field_rect.pad(field_padding_);
        return padded_field_rect.contains_point(robot_position);
    }
    return false;
}

void AgentActionClient::get_task() {
    auto optional_task =
        current_position_->get_task(last_world_state_, field_dimensions_, play_state_);
    if (optional_task.has_value()) {
        RobotIntent task = optional_task.value();

        // note that because these are our RobotIntent structs, this comparison
        // uses our custom struct overloads
        if (task != last_task_) {
            last_task_ = task;
            send_new_goal();
        }
    }
    current_state_publisher_->publish(rj_msgs::build<rj_msgs::msg::AgentState>().state(
        rj_convert::convert_to_ros(current_position_->get_current_state())));
}

void AgentActionClient::send_new_goal() {
    using namespace std::placeholders;

    if (!client_ptr_->wait_for_action_server()) {
        SPDLOG_ERROR("Action server not available after waiting");
        rclcpp::shutdown();
    }

    auto goal_msg = RobotMove::Goal();
    goal_msg.robot_intent = rj_convert::convert_to_ros(last_task_);

    auto send_goal_options = rclcpp_action::Client<RobotMove>::SendGoalOptions();
    send_goal_options.goal_response_callback = [this](auto arg) { goal_response_callback(arg); };
    send_goal_options.feedback_callback = [this](auto arg1, auto arg2) {
        feedback_callback(arg1, arg2);
    };
    send_goal_options.result_callback = [this](auto arg) { result_callback(arg); };
    client_ptr_->async_send_goal(goal_msg, send_goal_options);
}

// With the get_task function deleted, we need to initialize the new class once
// in the initializer. This will call the class which will analyze the
// situation based on the current tick.

void AgentActionClient::goal_response_callback(GoalHandleRobotMove::SharedPtr goal_handle) {
    if (!goal_handle) {
        current_position_->set_goal_canceled();
    }
}

void AgentActionClient::feedback_callback(
    GoalHandleRobotMove::SharedPtr, const std::shared_ptr<const RobotMove::Feedback> feedback) {
    double time_left = rj_convert::convert_from_ros(feedback->time_left).count();
    if (current_position_ == nullptr) {
        current_position_->set_time_left(time_left);
    }
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
}

void AgentActionClient::get_communication() {
    // Don't even humor requests from robots that aren't alive
    if (!check_robot_alive(robot_id_)) {
        return;
    }

    bool any_robots_alive = false;
    for (u_int8_t i = 0; i < kNumShells; i++) {
        if (check_robot_alive(i)) {
            any_robots_alive = true;
            break;
        }
    }

    if (!any_robots_alive) {
        return;
    }

    auto optional_communication_request = current_position_->send_communication_request();
    if (optional_communication_request.empty()) {
        return;
    }

    for (int i = 0; i < optional_communication_request.size(); i++) {
        auto communication_request = optional_communication_request.front();
        optional_communication_request.pop_front();

        // create a buffer to hold the responses and the outgoing request
        communication::AgentPosResponseWrapper buffered_response;

        auto request = std::make_shared<rj_msgs::srv::AgentCommunication::Request>();
        request->agent_request = rj_convert::convert_to_ros(communication_request.request);

        // send communication requests
        std::vector<u_int8_t> sent_robot_ids = {};
        if (communication_request.broadcast) {
            for (u_int8_t i = 0; i < kNumShells; i++) {
                if (i != robot_id_ && check_robot_alive(i)) {
                    robot_communication_cli_[i]->async_send_request(
                        request, [this, i](const std::shared_future<
                                           rj_msgs::srv::AgentCommunication::Response::SharedPtr>
                                               response) {
                            receive_response_callback(response, ((u_int8_t)i));
                        });
                    sent_robot_ids.push_back(i);
                }
            }
            // set broadcast to true in buffer
            buffered_response.broadcast = true;
        } else {
            for (u_int8_t i : communication_request.target_agents) {
                if (i != robot_id_ && check_robot_alive(i)) {
                    robot_communication_cli_[i]->async_send_request(
                        request,
                        [this, i](const std::shared_future<
                                  rj_msgs::srv::AgentCommunication::Response::SharedPtr>
                                      response) { receive_response_callback(response, i); });
                    sent_robot_ids.push_back(i);
                }
            }
        }

        buffered_response.to_robot_ids = sent_robot_ids;
        buffered_response.associated_request = communication_request.request;
        buffered_response.urgent = communication_request.urgent;
        buffered_response.created = RJ::now();
        buffered_responses_.push_back(buffered_response);
    }
}

void AgentActionClient::receive_communication_callback(
    const std::shared_ptr<rj_msgs::srv::AgentCommunication::Request>& request,
    const std::shared_ptr<rj_msgs::srv::AgentCommunication::Response>& response) {
    if (current_position_ == nullptr) {
        communication::AgentResponse agent_response;
        communication::AgentRequest agent_request =
            rj_convert::convert_from_ros(request->agent_request);
        communication::Acknowledge acknowledge{};
        communication::generate_uid(acknowledge);
        agent_response.associated_request = agent_request;
        agent_response.response = acknowledge;
        response->agent_response = rj_convert::convert_to_ros(agent_response);
    } else {
        // Convert agent request into AgentToPosCommRequest
        communication::AgentPosRequestWrapper agent_request;
        communication::AgentRequest received_request =
            rj_convert::convert_from_ros(request->agent_request);
        agent_request.request = received_request;

        // Give the current position the request and receive the response to send back
        communication::PosAgentResponseWrapper pos_to_agent_response =
            current_position_->receive_communication_request(agent_request);

        // Convert PosToAgentCommResponse into AgentResponse
        communication::AgentResponse agent_response{received_request,
                                                    pos_to_agent_response.response};
        response->agent_response = rj_convert::convert_to_ros(agent_response);
    }
}

void AgentActionClient::receive_response_callback(
    const std::shared_future<rj_msgs::srv::AgentCommunication::Response::SharedPtr>& response,
    u_int8_t robot_id) {
    // Convert response from other agent to c++
    communication::AgentResponse agent_response =
        rj_convert::convert_from_ros(response.get()->agent_response);

    for (u_int32_t i = 0; i < buffered_responses_.size(); i++) {
        if (buffered_responses_[i].associated_request == agent_response.associated_request) {
            // add the robot id in the corresponding (increasing) location in the received_robot_ids
            if (buffered_responses_[i].received_robot_ids.empty()) {
                buffered_responses_[i].received_robot_ids.push_back(robot_id);
                buffered_responses_[i].responses.push_back(agent_response.response);
            } else {
                for (u_int32_t j = 0; j < buffered_responses_[i].received_robot_ids.size(); j++) {
                    if (j == buffered_responses_[i].received_robot_ids.size() - 1) {
                        // The response should be added at the end of the buffer
                        buffered_responses_[i].received_robot_ids.push_back(robot_id);
                        buffered_responses_[i].responses.push_back(agent_response.response);
                        break;
                    } else if (buffered_responses_[i].received_robot_ids[j] > robot_id) {
                        // The response should be added at i in the buffer
                        buffered_responses_[i].received_robot_ids.insert(
                            buffered_responses_[i].received_robot_ids.begin() + j, robot_id);
                        buffered_responses_[i].responses.insert(
                            buffered_responses_[i].responses.begin() + j, agent_response.response);
                        break;
                    }
                }
            }

            // if the message is urgent -> relay the response
            // if we've received a response from all of the robots we want -> relay the response
            if (buffered_responses_[i].urgent ||
                (buffered_responses_[i].broadcast &&
                 buffered_responses_[i].received_robot_ids.size() >= 5) ||
                (buffered_responses_[i].received_robot_ids.size() ==
                 buffered_responses_[i].to_robot_ids.size())) {
                current_position_->receive_communication_response(buffered_responses_[i]);
                buffered_responses_.erase(buffered_responses_.begin() + i);
                return;
            }
        }
    }
}

void AgentActionClient::check_communication_timeout() {
    for (u_int32_t i = 0; i < buffered_responses_.size(); i++) {
        if (RJ::now() - buffered_responses_[i].created > timeout_duration_) {
            current_position_->receive_communication_response(buffered_responses_[i]);
            buffered_responses_.erase(buffered_responses_.begin() + i);
        }
    }
}

}  // namespace strategy

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    rclcpp::executors::MultiThreadedExecutor executor;

    // spin up one action client for each robot
    // (must be added to a vector so shared_ptrs aren't deleted when they go out of scope)
    std::list<strategy::AgentActionClient> agents;  // Linked List because no move operator in AAC
    for (int i = 0; i < 6;
         ++i) {  // TODO (Kevin): make this kNumShells and brick the non-used shells
        agents.emplace_back(i);
        auto& agent = agents.back();
        start_global_param_provider(agent.node().get(), kGlobalParamServerNode);
        executor.add_node(agent.node());
    }
    executor.spin();
    rclcpp::shutdown();
    return 0;
}