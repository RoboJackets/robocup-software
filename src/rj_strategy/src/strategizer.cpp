#pragma once
#include "rj_strategy/strategizer.hpp"

#include <chrono>
#include <thread>

#include "rj_strategy/commands/move_command.hpp"
#include "rj_strategy/commands/wait_command.hpp"

namespace strategy {
Strategizer::Strategizer() : Node("stratgizer") {
    client_ptr_ = rclcpp_action::create_client<rj_msgs::action::RobotMove>(this, "robot_move");

    scheduler = std::make_shared<commands::CommandScheduler>();
    commands::WaitCommand wCommand(20.);
    std::shared_ptr<commands::WaitCommand> wCommand1 =
        std::make_shared<commands::WaitCommand>(5000);
    // scheduler->scheduleCommand(wCommand1);
    auto thread_callback = [this]() { this->thread_callback(); };
    std::thread thread(thread_callback);

    for (int r_id = 0; r_id < 5; r_id++) {
        statePublishers.push_back(this->create_publisher<rj_msgs::msg::AgentState>(
            fmt::format("strategy/positon/robot_state/robot_{}", r_id), 1));
        // NOLINT
    }

    world_state_sub_ = this->create_subscription<rj_msgs::msg::WorldState>(
        ::vision_filter::topics::kWorldStateTopic, 1,
        [this](rj_msgs::msg::WorldState::SharedPtr msg) { world_state_callback(msg); });

    field_dimensions_sub_ = this->create_subscription<rj_msgs::msg::FieldDimensions>(
        "config/field_dimensions", rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::FieldDimensions::SharedPtr msg) { field_dimensions_callback(msg); });

    alive_robots_sub_ = this->create_subscription<rj_msgs::msg::AliveRobots>(
        ::radio::topics::kAliveRobotsTopic, 1,
        [this](rj_msgs::msg::AliveRobots::SharedPtr msg) { alive_robots_callback(msg); });

    game_settings_sub_ = this->create_subscription<rj_msgs::msg::GameSettings>(
        "config/game_settings", 1,
        [this](rj_msgs::msg::GameSettings::SharedPtr msg) { game_settings_callback(msg); });

    goalie_id_sub_ = this->create_subscription<rj_msgs::msg::Goalie>(
        ::referee::topics::kGoalieTopic, rclcpp::QoS(1).transient_local(),
        [this](rj_msgs::msg::Goalie::SharedPtr msg) { goalie_id_callback(msg->goalie_id); });

    using namespace std::chrono_literals;

    status_pub = this->create_publisher<std_msgs::msg::String>("strategizer", rclcpp::QoS(1));

    timer_ = this->create_wall_timer(17ms, std::bind(&Strategizer::timer_callback, this));
    commandTimer = this->create_wall_timer(17ms, std::bind(&Strategizer::tick, this));
    SPDLOG_INFO(
        "===============================================SPDLOG in Strategizer "
        "Constructor====================================");
    thread.join();
}
void Strategizer::thread_callback() {
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    planning::MotionCommand mCommand = planning::MotionCommand{"path_target",
                                                               planning::LinearMotionInstant{
                                                                   rj_geometry::Point{1.33, 2.29},
                                                                   rj_geometry::Point{0, 0},
                                                               },
                                                               planning::FaceTarget(), true};
    Robot robot1(1);

    std::shared_ptr<commands::MoveCommand> move_command =
        std::make_shared<commands::MoveCommand>(robot1, mCommand, client_ptr_);
    SPDLOG_INFO("===============================Built Motion Command==========================");
    if (scheduler->scheduleCommand(move_command)) {
        SPDLOG_INFO(
            "===============================Scheduled Motion Command==========================");
    } else {
        SPDLOG_INFO("===============================Scheduling Failed==========================");
    }
}
void Strategizer::tick() {
    // SPDLOG_INFO("===============================================Strategizer
    // tick====================================");
    scheduler.get()->tick();
}

void Strategizer::world_state_callback(const rj_msgs::msg::WorldState::SharedPtr& msg) {
    worldStateCache.push_back(msg);

    if (worldStateCache.size() > world_state_cache_max_size) {
        std::move(worldStateCache.front());
        worldStateCache.pop_front();
    }
}

void Strategizer::field_dimensions_callback(const rj_msgs::msg::FieldDimensions::SharedPtr& msg) {
    FieldDimensions field_dimensions = rj_convert::convert_from_ros(*msg);
    field_dimensions_ = field_dimensions;
}

void Strategizer::alive_robots_callback(const rj_msgs::msg::AliveRobots::SharedPtr& msg) {}

void Strategizer::game_settings_callback(const rj_msgs::msg::GameSettings::SharedPtr& msg) {}

void Strategizer::goalie_id_callback(int id) {}

void Strategizer::timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Strategizer is running";
    status_pub->publish(message);
}

}  // namespace strategy

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    rj_utils::set_spdlog_default_ros2("processor");

    auto node = std::make_shared<strategy::Strategizer>();
    // start_global_param_provider(node.get(), kGlobalParamServerNode);

    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
