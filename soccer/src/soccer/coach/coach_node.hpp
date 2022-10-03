//#pragma once

#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/play_state.hpp>
#include <rj_msgs/msg/coach.hpp>
#include <rj_msgs/msg/global_override.hpp>

#include <rj_geometry_msgs/msg/point.hpp>

#include "game_state.hpp"

enum match_situation {
    ball_placement,
    kickoff,
    free_kick,
    penalty_kick,
    in_play,
};

class CoachNode : public rclcpp::Node {
public:
    CoachNode(const rclcpp::NodeOptions& options);

private:
    rclcpp::Publisher<rj_msgs::msg::Coach>::SharedPtr coach_pub_;
    rclcpp::Subscription<rj_msgs::msg::PlayState>::SharedPtr playstate_sub_;
    rclcpp::TimerBase::SharedPtr playstate_change_timer_;

    rj_msgs::msg::PlayState current_play_state_;
    bool playstate_has_changed_ = true;

    void playstate_callback(rj_msgs::msg::PlayState::SharedPtr msg);
    void check_for_playstate_change();
};