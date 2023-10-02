#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_constants/constants.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry_msgs/msg/point.hpp>
#include <rj_msgs/msg/alive_robots.hpp>
#include <rj_msgs/msg/coach_state.hpp>
#include <rj_msgs/msg/game_settings.hpp>
#include <rj_msgs/msg/global_override.hpp>
#include <rj_msgs/msg/goalie.hpp>
#include <rj_msgs/msg/play_state.hpp>
#include <rj_msgs/msg/position_assignment.hpp>
#include <rj_msgs/msg/robot_state.hpp>
#include <rj_msgs/msg/robot_status.hpp>
#include <rj_msgs/msg/world_state.hpp>
#include <rj_utils/logging.hpp>

#include "game_state.hpp"
#include "strategy/agent/position/defense.hpp"
#include "strategy/agent/position/goalie.hpp"
#include "strategy/agent/position/offense.hpp"
#include "strategy/agent/position/position.hpp"
#include "strategy/agent/position/runner.hpp"

namespace strategy {
enum MatchSituation {
    ball_placement,  // ball placement on a restart restart.ball_placement
    kickoff,         // simple kickoff (start game/match/play)
    free_kick,       // either direct or indirect free kicks (direct and indirect are outdated as of
                     // 10/2022)
    penalty_kick,    // penalty kick restarts
    in_play,         // normal play
};

enum Positions { Goalie, Defense, Offense, PenaltyPlayer, GoalKicker, Runner};

// These values are explicitly declared because they are the ints that are published to
// strategy/positions i.e. the same values as strategy::Positions
namespace OverridePosition {
enum OverridePosition { Goalie = 0, Defense = 1, Offense = 2, None = 3, Runner = 4};
}  // namespace OverridePosition

/**
 * @brief This node takes the referee information and standardizes it for consumption of the
 * individual robot agents.
 *
 * The coach node subscribes to the /referee/playstate topic and translates this data
 * into pertinent information before sending it to another topic (/strategy/coach) for the robots
 * to use.
 */
class CoachNode : public rclcpp::Node {
public:
    CoachNode(const rclcpp::NodeOptions& options);

private:
    rclcpp::Publisher<rj_msgs::msg::CoachState>::SharedPtr coach_state_pub_;

    /*
     * Used to publish the defense areas as shapes for the planner node to recognize as obstacles
     * for non-goalies.
     */
    rclcpp::Publisher<rj_geometry_msgs::msg::ShapeSet>::SharedPtr def_area_obstacles_pub_;
    /*
     * Used to add the walls of the goals to the list of global obstacles.
     */
    rclcpp::Publisher<rj_geometry_msgs::msg::ShapeSet>::SharedPtr global_obstacles_pub_;

    /*
     * Information needed to calculate the static obstacles (defense areas and goal walls).
     */
    rclcpp::Subscription<rj_msgs::msg::FieldDimensions>::SharedPtr field_dimensions_sub_;

    /*
     * Let the referee determine which robot is the goalie.
     */
    rclcpp::Subscription<rj_msgs::msg::Goalie>::SharedPtr goalie_sub_;

    rclcpp::Publisher<rj_msgs::msg::PositionAssignment>::SharedPtr positions_pub_;
    rclcpp::Subscription<rj_msgs::msg::PositionAssignment>::SharedPtr overrides_sub_;
    rclcpp::Subscription<rj_msgs::msg::PlayState>::SharedPtr play_state_sub_;
    rclcpp::Subscription<rj_msgs::msg::WorldState>::SharedPtr world_state_sub_;
    rclcpp::Subscription<rj_msgs::msg::RobotStatus>::SharedPtr robot_status_subs_[kNumShells];
    rclcpp::TimerBase::SharedPtr coach_action_callback_timer_;

    // Subscriber to get the alive robots for non-simulated games
    rclcpp::Subscription<rj_msgs::msg::AliveRobots>::SharedPtr alive_robots_sub_;
    // Subscriber to determine whether the game mode is simulated or real play
    rclcpp::Subscription<rj_msgs::msg::GameSettings>::SharedPtr game_settings_sub_;

    rj_msgs::msg::PlayState current_play_state_;
    bool possessing_ = false;
    bool play_state_has_changed_ = true;

    FieldDimensions current_field_dimensions_;
    bool have_field_dimensions_ = false;

    /*
     * Overrides from the UI.
     */
    std::array<uint32_t, kNumShells> current_overrides_;
    bool have_overrides_ = false;

    int goalie_id_{0};

    void play_state_callback(const rj_msgs::msg::PlayState::SharedPtr msg);
    void world_state_callback(const rj_msgs::msg::WorldState::SharedPtr msg);
    void ball_sense_callback(const rj_msgs::msg::RobotStatus::SharedPtr msg, bool our_team);
    void field_dimensions_callback(const rj_msgs::msg::FieldDimensions::SharedPtr& msg);
    void goalie_callback(const rj_msgs::msg::Goalie::SharedPtr& msg);
    void overrides_callback(const rj_msgs::msg::PositionAssignment::SharedPtr& msg);
    void check_for_play_state_change();
    void alive_robots_callback(const rj_msgs::msg::AliveRobots::SharedPtr& msg);
    void game_settings_callback(const rj_msgs::msg::GameSettings::SharedPtr& msg);
    /*
     * Handles actions the Coach does every tick. Currently calls assign_positions() and
     * check_for_play_state_change.
     */
    void coach_ticker();
    /*
     * Assigns Positions to robots with IDs 1 through 16, depending on current possession.
     * Publishes new Position to the topic /strategy/Position (message type PositionAssignment)
     */
    void assign_positions();

    // Helpers for assign_positions
    void assign_positions_penalty(std::array<uint32_t, kNumShells>& positions);
    void assign_positions_kickoff(std::array<uint32_t, kNumShells>& positions);
    void assign_positions_freekick(std::array<uint32_t, kNumShells>& positions);
    void assign_positions_normal(std::array<uint32_t, kNumShells>& positions);

    /*
     * Publishes the static obstacles.
     */
    void publish_static_obstacles();

    /*
     * Calculates the defense area as a set of obstacles. The defense area, per the rules, is the
     * box in front of each goal where only that team's goalie can be in and touch the ball.
     */
    rj_geometry::ShapeSet create_defense_area_obstacles();

    /*
     * Calculates the physical (3D) walls of the each goal as a set of obstacles, so robots don't
     * crash into them.
     */
    rj_geometry::ShapeSet create_goal_wall_obstacles();

    std::vector<u_int8_t> alive_robots_ = {};
    bool is_simulated_ = false;
    static constexpr double field_padding_ = 0.3;

    /**
     * @brief checks whether a robot is visible, in the field, and (if the game is not
     * simulated) whether or not the robot is in the alive robots list.
     *
     * @param robot_id the id of the robot to check alive
     * @return true if robot is connected, visible, and near the field
     * @return false if the robot is not connected or is not visible
     */
    bool check_robot_alive(u_int8_t robot_id);
};

}  // namespace strategy
