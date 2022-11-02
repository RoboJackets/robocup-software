#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_constants/constants.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry_msgs/msg/point.hpp>
#include <rj_msgs/msg/coach_state.hpp>
#include <rj_msgs/msg/global_override.hpp>
#include <rj_msgs/msg/play_state.hpp>
#include <rj_msgs/msg/robot_state.hpp>
#include <rj_msgs/msg/robot_status.hpp>
#include <rj_msgs/msg/world_state.hpp>

#include "game_state.hpp"

namespace strategy {
enum MatchSituation {
    ball_placement,  // ball placement on a restart restart.ball_placement
    kickoff,         // simple kickoff (start game/match/play)
    free_kick,       // either direct or indirect free kicks (direct and indirect are outdated as of
                     // 10/2022)
    penalty_kick,    // penalty kick restarts
    in_play,         // normal play
};

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
    rclcpp::Subscription<rj_msgs::msg::PlayState>::SharedPtr play_state_sub_;
    rclcpp::Subscription<rj_msgs::msg::WorldState>::SharedPtr world_state_sub_;
    rclcpp::Subscription<rj_msgs::msg::RobotStatus>::SharedPtr robot_status_subs_[kNumShells];
    rclcpp::TimerBase::SharedPtr play_state_change_timer_;

    rj_msgs::msg::PlayState current_play_state_;
    bool possessing_ = false;
    bool play_state_has_changed_ = true;

    void play_state_callback(const rj_msgs::msg::PlayState::SharedPtr msg);
    void world_state_callback(const rj_msgs::msg::WorldState::SharedPtr msg);
    void ball_sense_callback(const rj_msgs::msg::RobotStatus::SharedPtr msg, bool our_team);
    void check_for_play_state_change();
};

}  // namespace strategy
