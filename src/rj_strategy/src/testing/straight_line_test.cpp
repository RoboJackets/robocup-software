#include "rj_strategy/testing/straight_line_test.hpp"

namespace strategy {
using RobotMove = rj_msgs::action::RobotMove;
using GoalHandleRobotMove = rclcpp_action::ClientGoalHandle<RobotMove>;

StraightLineTest::StraightLineTest() : StraightLineTest(0) {}

StraightLineTest::StraightLineTest(int r_id)
    : rclcpp::Node(::fmt::format("agent_{}_straight_line_node", r_id),
                   rclcpp::NodeOptions{}
                       .automatically_declare_parameters_from_overrides(true)
                       .allow_undeclared_parameters(true)),
      current_position_{std::make_unique<Line>(r_id)},
      robot_id_{r_id} {
    client_ptr_ = rclcpp_action::create_client<RobotMove>(this, "robot_move");

    current_state_publisher_ = create_publisher<AgentStateMsg>(
        fmt::format("strategy/position/robot_state/robot_{}", r_id), 1);

    world_state_sub_ = create_subscription<rj_msgs::msg::WorldState>(
        ::vision_filter::topics::kWorldStateTopic, 1,
        [this](const rj_msgs::msg::WorldState::SharedPtr msg) { world_state_callback(msg); });

    play_state_sub_ = create_subscription<rj_msgs::msg::PlayState>(
        ::referee::topics::kPlayStateTopic, 1,
        [this](const rj_msgs::msg::PlayState::SharedPtr msg) { play_state_callback(msg); });

    field_dimensions_sub_ = create_subscription<rj_msgs::msg::FieldDimensions>(
        "config/field_dimensions", rclcpp::QoS(1).transient_local(),
        [this](const rj_msgs::msg::FieldDimensions::SharedPtr msg) {
            field_dimensions_callback(msg);
        });

    alive_robots_sub_ = create_subscription<rj_msgs::msg::AliveRobots>(
        ::radio::topics::kAliveRobotsTopic, 1,
        [this](const rj_msgs::msg::AliveRobots::SharedPtr msg) { alive_robots_callback(msg); });

    game_settings_sub_ = create_subscription<rj_msgs::msg::GameSettings>(
        "config/game_settings", 1,
        [this](const rj_msgs::msg::GameSettings::SharedPtr msg) { game_settings_callback(msg); });

    line_direction_sub_ = create_subscription<std_msgs::msg::Bool>(
        "line_direction", 1,
        [this](const std_msgs::msg::Bool::SharedPtr msg) { line_direction_callback(msg); });

    int hz = 10;
    get_task_timer_ = create_wall_timer(std::chrono::milliseconds(1000 / hz),
                                        std::bind(&StraightLineTest::get_task, this));
}

void StraightLineTest::world_state_callback(const rj_msgs::msg::WorldState::SharedPtr& msg) {
    WorldState world_state = rj_convert::convert_from_ros(*msg);
    auto lock = std::lock_guard(world_state_mutex_);
    last_world_state_ = std::move(world_state);
}

void StraightLineTest::play_state_callback(const rj_msgs::msg::PlayState::SharedPtr& msg) {
    PlayState play_state = rj_convert::convert_from_ros(*msg);
    play_state_ = play_state;
    current_position_->update_play_state(play_state);
}

void StraightLineTest::field_dimensions_callback(
    const rj_msgs::msg::FieldDimensions::SharedPtr& msg) {
    FieldDimensions field_dimensions = rj_convert::convert_from_ros(*msg);
    field_dimensions_ = field_dimensions;
    current_position_->update_field_dimensions(field_dimensions);
}

void StraightLineTest::alive_robots_callback(const rj_msgs::msg::AliveRobots::SharedPtr& msg) {
    alive_robots_ = msg->alive_robots;
    current_position_->update_alive_robots(alive_robots_);
}

void StraightLineTest::game_settings_callback(const rj_msgs::msg::GameSettings::SharedPtr& msg) {
    is_simulated_ = msg->simulation;
}

void StraightLineTest::line_direction_callback(const std_msgs::msg::Bool::SharedPtr& msg) {
    if (msg->data != vertical_) {
        vertical_ = msg->data;
        current_position_ = std::make_unique<Line>(robot_id_, vertical_);
    }
}

bool StraightLineTest::check_robot_alive(uint8_t robot_id) {
    if (!is_simulated_) {
        return alive_robots_.at(robot_id);
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

void StraightLineTest::get_task() {
    auto lock = std::lock_guard(world_state_mutex_);

    auto optional_task =
        current_position_->get_task(last_world_state_, field_dimensions_, play_state_);

    if (optional_task.has_value()) {
        RobotIntent task = optional_task.value();

        if (task != last_task_) {
            last_task_ = task;
            send_new_goal();
        }
    }

    current_state_publisher_->publish(rj_msgs::build<rj_msgs::msg::AgentState>().state(
        rj_convert::convert_to_ros(current_position_->get_current_state())));
}

void StraightLineTest::send_new_goal() {
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

[[nodiscard]] WorldState* StraightLineTest::world_state() {
    auto lock = std::lock_guard(world_state_mutex_);
    return &last_world_state_;
}

void StraightLineTest::goal_response_callback(GoalHandleRobotMove::SharedPtr goal_handle) {
    if (!goal_handle) {
        current_position_->set_goal_canceled();
    }
}

void StraightLineTest::feedback_callback(
    GoalHandleRobotMove::SharedPtr, const std::shared_ptr<const RobotMove::Feedback> feedback) {
    double time_left = rj_convert::convert_from_ros(feedback->time_left).count();
    current_position_->set_time_left(time_left);
}

void StraightLineTest::result_callback(const GoalHandleRobotMove::WrappedResult& result) {
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
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

}  // namespace strategy

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    rclcpp::executors::MultiThreadedExecutor executor;

    std::vector<rclcpp::Node::SharedPtr> agents;
    for (int i = 0; i < 6; i++) {
        auto agent = std::make_shared<strategy::StraightLineTest>(i);
        start_global_param_provider(agent.get(), kGlobalParamServerNode);
        agents.push_back(agent);
    }
    for (const auto& agent : agents) {
        executor.add_node(agent);
    }
    executor.spin();
}