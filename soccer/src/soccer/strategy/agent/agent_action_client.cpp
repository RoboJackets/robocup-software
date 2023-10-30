#include "agent_action_client.hpp"

#include "rj_constants/topic_names.hpp"

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
    // create a ptr to ActionClient
    client_ptr_ = rclcpp_action::create_client<RobotMove>(this, "robot_move");

    world_state_sub_ = create_subscription<rj_msgs::msg::WorldState>(
        ::vision_filter::topics::kWorldStateTopic, 1,
        [this](rj_msgs::msg::WorldState::SharedPtr msg) { world_state_callback(msg); });

    coach_state_sub_ = create_subscription<rj_msgs::msg::CoachState>(
        topics::kCoachStateTopic, 1,
        [this](rj_msgs::msg::CoachState::SharedPtr msg) { coach_state_callback(msg); });

    field_dimensions_sub_ = create_subscription<rj_msgs::msg::FieldDimensions>(
        "config/field_dimensions", 1,
        [this](rj_msgs::msg::FieldDimensions::SharedPtr msg) { field_dimensions_callback(msg); });

    alive_robots_sub_ = create_subscription<rj_msgs::msg::AliveRobots>(
        "strategy/alive_robots", 1,
        [this](rj_msgs::msg::AliveRobots::SharedPtr msg) { alive_robots_callback(msg); });

    game_settings_sub_ = create_subscription<rj_msgs::msg::GameSettings>(
        "config/game_settings", 1,
        [this](rj_msgs::msg::GameSettings::SharedPtr msg) { game_settings_callback(msg); });

    robot_communication_srv_ = create_service<rj_msgs::srv::AgentCommunication>(
        fmt::format("agent_{}_incoming", r_id),
        [this](const std::shared_ptr<rj_msgs::srv::AgentCommunication::Request> request,
               std::shared_ptr<rj_msgs::srv::AgentCommunication::Response> response) {
            receive_communication_callback(request, response);
        });

    // Create clients
    for (size_t i = 0; i < kNumShells; i++) {
        robot_communication_cli_[i] =
            create_client<rj_msgs::srv::AgentCommunication>(fmt::format("agent_{}_incoming", i));
    }

    positions_sub_ = create_subscription<rj_msgs::msg::PositionAssignment>(
        topics::kPositionsTopic, 1,
        [this](rj_msgs::msg::PositionAssignment::SharedPtr msg) { update_position(msg); });

    // TODO(Kevin): make ROS param for this
    int hz = 10;
    get_task_timer_ = create_wall_timer(std::chrono::milliseconds(1000 / hz),
                                        std::bind(&AgentActionClient::get_task, this));

    // TODO(Kevin): make ROS param for this
    int agent_communication_hz = 60;
    get_communication_timer_ =
        create_wall_timer(std::chrono::milliseconds(1000 / agent_communication_hz), [this]() {
            get_communication();
            check_communication_timeout();
        });

    update_alive_robots_timer_ = create_wall_timer(std::chrono::milliseconds(1000),
                                                   [this]() { update_position_alive_robots(); });

    if (r_id == 0) {
        current_position_ = std::make_unique<Goalie>(r_id);
    } else if (r_id == 1) {
        current_position_ = std::make_unique<Runner>(r_id);
    } else if (r_id == 3 || r_id == 5) {
        current_position_ = std::make_unique<Defense>(r_id);
    } else if (r_id == 2 || r_id == 4) {
        current_position_ = std::make_unique<Offense>(r_id);
    }
}

void AgentActionClient::world_state_callback(const rj_msgs::msg::WorldState::SharedPtr& msg) {
    if (current_position_ == nullptr) {
        return;
    }

    WorldState world_state = rj_convert::convert_from_ros(*msg);
    current_position_->update_world_state(world_state);
    // avoid mutex issues w/ world state (probably not an issue in AC, but
    // already here so why not)
    auto lock = std::lock_guard(world_state_mutex_);
    last_world_state_ = std::move(world_state);
}

void AgentActionClient::coach_state_callback(const rj_msgs::msg::CoachState::SharedPtr& msg) {
    if (current_position_ == nullptr) {
        return;
    }

    current_position_->update_coach_state(*msg);
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

void AgentActionClient::alive_robots_callback(const rj_msgs::msg::AliveRobots::SharedPtr& msg) {
    alive_robots_ = msg->alive_robots;
}

void AgentActionClient::game_settings_callback(const rj_msgs::msg::GameSettings::SharedPtr& msg) {
    is_simulated_ = msg->simulation;
}

bool AgentActionClient::check_robot_alive(u_int8_t robot_id) {
    if (!is_simulated_) {
        return std::find(alive_robots_.begin(), alive_robots_.end(), robot_id) !=
               alive_robots_.end();
    } else {
        if (this->world_state()->get_robot(true, robot_id).visible) {
            rj_geometry::Point robot_position =
                this->world_state()->get_robot(true, robot_id).pose.position();
            rj_geometry::Rect padded_field_rect = field_dimensions_.field_coordinates();
            padded_field_rect.pad(field_padding_);
            return padded_field_rect.contains_point(robot_position);
        }
        return false;
    }
}

void AgentActionClient::get_task() {
    // Initialize default positions (if not already initialized)
    if (current_position_ == nullptr) {
        if (robot_id_ == 0) {
            current_position_ = std::make_unique<Goalie>(robot_id_);
        } else if (robot_id_ == 1) {
            current_position_ = std::make_unique<Runner>(robot_id_);
        } else if (robot_id_ == 2) {
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
            last_task_ = task;
            send_new_goal();
        }
    }
}

void AgentActionClient::update_position(const rj_msgs::msg::PositionAssignment::SharedPtr& msg) {
    std::unique_ptr<Position> next_position_;
    switch (msg->client_positions[robot_id_]) {
        case 0:
            next_position_ = std::make_unique<Goalie>(robot_id_);
            break;
        case 1:
            next_position_ = std::make_unique<Runner>(robot_id_);
            break;
        case 2:
            next_position_ = std::make_unique<Offense>(robot_id_);
            break;
        case 3:
            next_position_ = std::make_unique<Defense>(robot_id_);
            break;
        case 4:
            next_position_ = std::make_unique<GoalKicker>(robot_id_);
            break;
    }

    if (current_position_ == nullptr) {
        current_position_ = std::move(next_position_);
    } else if (next_position_->get_name() != current_position_->get_name()) {
        // When a disconnected robot is found, notify the position
        current_position_->die();
        // Send any final death communications from robot
        get_communication();
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
    // Initialize default positions (if not already initialized)
    if (current_position_ == nullptr) {
        if (robot_id_ == 0) {
            current_position_ = std::make_unique<Goalie>(robot_id_);
        } else if (robot_id_ == 1) {
            current_position_ = std::make_unique<Runner>(robot_id_);
        } else if (robot_id_ == 2) {
            current_position_ = std::make_unique<Defense>(robot_id_);
        } else {
            current_position_ = std::make_unique<Offense>(robot_id_);
        }
    }

    auto goal_handle = future.get();
    if (!goal_handle) {
        current_position_->set_goal_canceled();
    }
}

void AgentActionClient::feedback_callback(
    GoalHandleRobotMove::SharedPtr, const std::shared_ptr<const RobotMove::Feedback> feedback) {
    // Initialize default positions (if not already initialized)
    if (current_position_ == nullptr) {
        if (robot_id_ == 0) {
            current_position_ = std::make_unique<Goalie>(robot_id_);
        } else if (robot_id_ == 1) {
            current_position_ = std::make_unique<Runner>(robot_id_);
        } else if (robot_id_ == 2) {
            current_position_ = std::make_unique<Defense>(robot_id_);
        } else {
            current_position_ = std::make_unique<Offense>(robot_id_);
        }
    }

    double time_left = rj_convert::convert_from_ros(feedback->time_left).count();
    current_position_->set_time_left(time_left);
}

void AgentActionClient::result_callback(const GoalHandleRobotMove::WrappedResult& result) {
    // Initialize default positions (if not already initialized)
    if (current_position_ == nullptr) {
        if (robot_id_ == 0) {
            current_position_ = std::make_unique<Goalie>(robot_id_);
        } else if (robot_id_ == 1) {
            current_position_ = std::make_unique<Runner>(robot_id_);
        } else if (robot_id_ == 2) {
            current_position_ = std::make_unique<Defense>(robot_id_);
        } else {
            current_position_ = std::make_unique<Offense>(robot_id_);
        }
    }

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

    // Initialize default positions (if not already initialized)
    if (current_position_ == nullptr) {
        if (robot_id_ == 0) {
            current_position_ = std::make_unique<Goalie>(robot_id_);
        } else if (robot_id_ == 1) {
            current_position_ = std::make_unique<Runner>(robot_id_);
        } else if (robot_id_ == 2) {
            current_position_ = std::make_unique<Defense>(robot_id_);
        } else {
            current_position_ = std::make_unique<Offense>(robot_id_);
        }
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
    if (!optional_communication_request.has_value()) {
        return;
    }

    auto communication_request = optional_communication_request.value();

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
                    request, [this, i](const std::shared_future<
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
    // Initialize default positions (if not already initialized)
    if (current_position_ == nullptr) {
        if (robot_id_ == 0) {
            current_position_ = std::make_unique<Goalie>(robot_id_);
        } else if (robot_id_ == 1) {
            current_position_ = std::make_unique<Runner>(robot_id_);
        } else if (robot_id_ == 2) {
            current_position_ = std::make_unique<Defense>(robot_id_);
        } else {
            current_position_ = std::make_unique<Offense>(robot_id_);
        }
    }

    for (u_int32_t i = 0; i < buffered_responses_.size(); i++) {
        if (RJ::now() - buffered_responses_[i].created > timeout_duration_) {
            current_position_->receive_communication_response(buffered_responses_[i]);
            buffered_responses_.erase(buffered_responses_.begin() + i);
        }
    }
}

[[nodiscard]] WorldState* AgentActionClient::world_state() {
    // thread-safe getter for world_state
    auto lock = std::lock_guard(world_state_mutex_);
    return &last_world_state_;
}

void AgentActionClient::update_position_alive_robots() {
    if (current_position_ == nullptr) {
        return;
    }

    if (!is_simulated_) {
        current_position_->update_alive_robots(alive_robots_);
    } else {
        std::vector<u_int8_t> alive_robots = {};
        for (u_int8_t i = 0; i < kNumShells; i++) {
            if (check_robot_alive(i)) {
                alive_robots.push_back(i);
            }
        }
        current_position_->update_alive_robots(std::move(alive_robots));
    }
}

}  // namespace strategy
