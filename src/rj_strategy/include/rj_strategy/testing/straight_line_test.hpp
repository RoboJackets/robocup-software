#pragma once

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <spdlog/spdlog.h>

#include <rj_common/game_state.hpp>
#include <rj_common/robot_intent.hpp>
#include <rj_constants/topic_names.hpp>
#include <rj_geometry/point.hpp>
#include <rj_msgs/action/robot_move.hpp>
#include <rj_msgs/msg/agent_state.hpp>
#include <rj_msgs/msg/alive_robots.hpp>
#include <rj_msgs/msg/field_dimensions.hpp>
#include <rj_msgs/msg/game_settings.hpp>
#include <rj_msgs/msg/line_test.hpp>
#include <rj_msgs/msg/play_state.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <rj_param_utils/global_params.hpp>
#include <rj_utils/logging.hpp>
#include <std_msgs/msg/bool.hpp>

#include "rj_strategy/agent/position.hpp"
#include "rj_strategy/agent/position/line.hpp"

// run test by using the command `make run-sim-line-test`
// Note: The the line can be changed by running following command in another terminal window:
// Make sure to source bash and ros in other window
// `ros2 topic pub -1 line rj_msgs/msg/LineTest "{pt: [{x: 0, y: 0}, {x: 1, y: 1}], r_id: 1}"`

DECLARE_FLOAT64("straight_line_test", start_x);
DECLARE_FLOAT64("straight_line_test", start_y);
DECLARE_FLOAT64("straight_line_test", end_x);
DECLARE_FLOAT64("straight_line_test", end_y);
DECLARE_FLOAT64("straight_line_test", robot_id);

namespace strategy {

class StraightLineTest : public rclcpp::Node {
public:
    using AgentStateMsg = rj_msgs::msg::AgentState;
    using RobotMove = rj_msgs::action::RobotMove;
    using GoalHandleRobotMove = rclcpp_action::ClientGoalHandle<RobotMove>;

    StraightLineTest();
    StraightLineTest(int r_id);
    ~StraightLineTest() = default;

private:
    // ros subscriptions
    rclcpp::Subscription<rj_msgs::msg::WorldState>::SharedPtr world_state_sub_;
    rclcpp::Subscription<rj_msgs::msg::FieldDimensions>::SharedPtr field_dimensions_sub_;
    rclcpp::Subscription<rj_msgs::msg::GameSettings>::SharedPtr game_settings_sub_;
    rclcpp::Subscription<rj_msgs::msg::PlayState>::SharedPtr play_state_sub_;
    rclcpp::Subscription<rj_msgs::msg::AliveRobots>::SharedPtr alive_robots_sub_;
    rclcpp::Subscription<rj_msgs::msg::LineTest>::SharedPtr line_direction_sub_;

    // subscription callbacks
    void world_state_callback(const rj_msgs::msg::WorldState::SharedPtr& msg);
    void play_state_callback(const rj_msgs::msg::PlayState::SharedPtr& msg);
    void alive_robots_callback(const rj_msgs::msg::AliveRobots::SharedPtr& msg);
    void field_dimensions_callback(const rj_msgs::msg::FieldDimensions::SharedPtr& msg);
    void game_settings_callback(const rj_msgs::msg::GameSettings::SharedPtr& msg);
    void line_direction_callback(const rj_msgs::msg::LineTest::SharedPtr& msg);

    rclcpp::Publisher<AgentStateMsg>::SharedPtr current_state_publisher_;

    // Ros ActionClient spec, for calls to planning ActionServer
    rclcpp_action::Client<RobotMove>::SharedPtr client_ptr_;
    /**
     * Send a new long term goal to the robot movement path planner
     */
    void goal_response_callback(GoalHandleRobotMove::SharedPtr future);
    /**
     * Retrieve the amount of time left for the current trajectory goal
     */
    void feedback_callback(GoalHandleRobotMove::SharedPtr,
                           const std::shared_ptr<const RobotMove::Feedback> feedback);
    /**
     * Tell the current line position whether it has finished moving
     */
    void result_callback(const GoalHandleRobotMove::WrappedResult& result);

    /**
     * Send a new goal to the path planner
     */
    void send_new_goal();

    std::unique_ptr<Position> current_position_;

    /**
     * Get the task the agent should be running (based on the current line position)
     */
    void get_task();
    rclcpp::TimerBase::SharedPtr get_task_timer_;

    RobotIntent last_task_;

    FieldDimensions field_dimensions_;
    PlayState play_state_ = PlayState::halt();
    std::array<bool, kNumShells> alive_robots_{};
    bool is_simulated_ = false;
    static constexpr double field_padding_ = 0.3;

    /**
     * Check whether a given robot is alive
     */
    bool check_robot_alive(uint8_t robot_id);

    const int robot_id_;

    [[nodiscard]] WorldState* world_state();
    WorldState last_world_state_;
    mutable std::mutex world_state_mutex_;
    rj_geometry::Point start_;
    rj_geometry::Point end_;
    uint8_t target_robot_id_;
};  // class StraightLineTest

}  // namespace strategy
