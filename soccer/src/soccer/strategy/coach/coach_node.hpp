#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_constants/constants.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry_msgs/msg/point.hpp>
#include <rj_msgs/msg/coach_state.hpp>
#include <rj_msgs/msg/global_override.hpp>
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

namespace strategy {
enum MatchSituation {
    ball_placement,  // ball placement on a restart restart.ball_placement
    kickoff,         // simple kickoff (start game/match/play)
    free_kick,       // either direct or indirect free kicks (direct and indirect are outdated as of
                     // 10/2022)
    penalty_kick,    // penalty kick restarts
    in_play,         // normal play
};

enum Positions { Goalie, Defense, Offense };

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

    rclcpp::Publisher<rj_geometry_msgs::msg::ShapeSet>::SharedPtr def_area_obstacles_pub_;
    rclcpp::Publisher<rj_geometry_msgs::msg::ShapeSet>::SharedPtr global_obstacles_pub_;

    rclcpp::Subscription<rj_msgs::msg::FieldDimensions>::SharedPtr field_dimensions_sub_;

    rclcpp::Publisher<rj_msgs::msg::PositionAssignment>::SharedPtr positions_pub_;
    rclcpp::Subscription<rj_msgs::msg::PlayState>::SharedPtr play_state_sub_;

    rclcpp::Subscription<rj_msgs::msg::WorldState>::SharedPtr world_state_sub_;
    rclcpp::Subscription<rj_msgs::msg::RobotStatus>::SharedPtr robot_status_subs_[kNumShells];
    rclcpp::TimerBase::SharedPtr coach_action_callback_timer_;

    rj_msgs::msg::PlayState current_play_state_;
    bool field_updated_ = false;
    bool possessing_ = false;
    bool play_state_has_changed_ = true;

    rj_msgs::msg::FieldDimensions current_field_dimensions_;
    bool have_field_dimensions_ = false;

    void play_state_callback(const rj_msgs::msg::PlayState::SharedPtr msg);
    void world_state_callback(const rj_msgs::msg::WorldState::SharedPtr msg);
    void ball_sense_callback(const rj_msgs::msg::RobotStatus::SharedPtr msg, bool our_team);
    void field_dimensions_callback(const rj_msgs::msg::FieldDimensions::SharedPtr& msg);
    void check_for_play_state_change();
    /*
     * Handles actions the Coach does every tick. Currently calls assign_positions and
     * check_for_play_state_change.
     */
    void coach_ticker();
    /*
     * Assigns positions to robots with IDs 1 through 16, depending on current possession.
     * Publishes new positions to the topic /strategy/positions (message type PositionAssignment)
     */
    void assign_positions();

    /*
     * Publishes the static obstacles.
     */
    void publish_static_obstacles();
};

}  // namespace strategy
