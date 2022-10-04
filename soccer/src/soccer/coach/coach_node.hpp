#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_geometry_msgs/msg/point.hpp>
#include <rj_msgs/msg/coach_state_interpretation.hpp>
#include <rj_msgs/msg/global_override.hpp>
#include <rj_msgs/msg/play_state.hpp>

#include "game_state.hpp"

enum MatchSituation {
    ball_placement,  // ball placement on a restart restart.ball_placement
    kickoff,         // simple kickoff (start game/match/play)
    free_kick,     // either direct or indirect free kicks (direct and indirect are outdated terms)
    penalty_kick,  // penalty kick restarts
    in_play,       // normal play
};

/**
 * @brief The coach node subscribes to the /referee/playstate topic and translates this data
 * into pertinent information before sending it to another topic (/strategy/coach) for the robots
 * to use.
 *
 * In short, this node takes the referee information and standardizes it for consumption of the
 * robots.
 *
 */
class CoachNode : public rclcpp::Node {
public:
    CoachNode(const rclcpp::NodeOptions& options);

private:
    rclcpp::Publisher<rj_msgs::msg::CoachStateInterpretation>::SharedPtr coach_pub_;
    rclcpp::Subscription<rj_msgs::msg::PlayState>::SharedPtr playstate_sub_;
    rclcpp::TimerBase::SharedPtr playstate_change_timer_;

    rj_msgs::msg::PlayState current_play_state_;
    bool playstate_has_changed_ = true;

    void playstate_callback(rj_msgs::msg::PlayState::SharedPtr msg);
    void check_for_playstate_change();
};