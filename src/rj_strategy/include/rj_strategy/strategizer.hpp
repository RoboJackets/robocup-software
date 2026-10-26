#pragma once
#include <deque>
#include <functional>
#include <future>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_common/game_state.hpp>
#include <rj_common/robot_intent.hpp>
#include <rj_common/time.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_msgs/action/robot_move.hpp>
#include <rj_msgs/msg/agent_state.hpp>
#include <rj_msgs/msg/alive_robots.hpp>
#include <rj_msgs/msg/field_dimensions.hpp>
#include <rj_msgs/msg/game_settings.hpp>
#include <rj_msgs/msg/goalie.hpp>
#include <rj_msgs/msg/override_position.hpp>
#include <rj_msgs/msg/play_state.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <rj_param_utils/global_params.hpp>
#include <rj_utils/logging.hpp>
#include <std_msgs/msg/string.hpp>

#include "rj_strategy/agent/position.hpp"
#include "rj_strategy/agent/position/robot_factory_position.hpp"
#include "rj_strategy/command.hpp"
#include "rj_strategy/command_scheduler.hpp"

// Communication
#include <rj_msgs/msg/acknowledge.hpp>
#include <rj_msgs/msg/agent_request.hpp>
#include <rj_msgs/srv/agent_communication.hpp>

#include "rj_strategy/agent/communication.hpp"

namespace strategy {
class Strategizer : public rclcpp::Node {
public:
    Strategizer();

private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::TimerBase::SharedPtr commandTimer;

    std::shared_ptr<commands::CommandScheduler> scheduler;

    std::vector<rclcpp::Publisher<rj_msgs::msg::AgentState>::SharedPtr> statePublishers;
    std::vector<rclcpp::Subscription<rj_msgs::msg::OverridePosition>::SharedPtr>
        overrideSubscriptions;

    // // ROS pub/subs
    rclcpp_action::Client<rj_msgs::action::RobotMove>::SharedPtr client_ptr_;
    rclcpp::Subscription<rj_msgs::msg::WorldState>::SharedPtr world_state_sub_;
    rclcpp::Subscription<rj_msgs::msg::FieldDimensions>::SharedPtr field_dimensions_sub_;
    rclcpp::Subscription<rj_msgs::msg::AliveRobots>::SharedPtr alive_robots_sub_;
    rclcpp::Subscription<rj_msgs::msg::GameSettings>::SharedPtr game_settings_sub_;
    rclcpp::Subscription<rj_msgs::msg::Goalie>::SharedPtr goalie_id_sub_;

    void world_state_callback(const rj_msgs::msg::WorldState::SharedPtr& msg);
    void field_dimensions_callback(const rj_msgs::msg::FieldDimensions::SharedPtr& msg);
    void alive_robots_callback(const rj_msgs::msg::AliveRobots::SharedPtr& msg);
    void game_settings_callback(const rj_msgs::msg::GameSettings::SharedPtr& msg);
    void test_play_callback(const rj_msgs::msg::OverridePosition::SharedPtr& message);
    void goalie_id_callback(int goalie_id);

    void timer_callback();

    void tick();

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub;

    int world_state_cache_max_size = 10;
    std::deque<rj_msgs::msg::WorldState::SharedPtr> worldStateCache;

    FieldDimensions field_dimensions_;
};

}  // namespace strategy