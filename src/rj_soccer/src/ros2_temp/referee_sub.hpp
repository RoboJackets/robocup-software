#pragma once

#include <rclcpp/executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rj_msgs/msg/goalie.hpp>
#include <rj_msgs/msg/match_state.hpp>
#include <rj_msgs/msg/play_state.hpp>
#include <rj_msgs/msg/raw_protobuf.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <rj_msgs/msg/team_info.hpp>

#include "context.hpp"
#include "node.hpp"

namespace ros2_temp {

using RawProtobufMsg = rj_msgs::msg::RawProtobuf;
using PlayStateMsg = rj_msgs::msg::PlayState;
using MatchStateMsg = rj_msgs::msg::MatchState;
using GoalieMsg = rj_msgs::msg::Goalie;
using TeamColorMsg = rj_msgs::msg::TeamColor;
using TeamInfoMsg = rj_msgs::msg::TeamInfo;

class RefereeSub : public Node {
public:
    RefereeSub(Context* context, rclcpp::Executor* executor);

    void run() override {}

private:
    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Subscription<RawProtobufMsg>::SharedPtr raw_sub_;
    rclcpp::Subscription<PlayStateMsg>::SharedPtr play_state_sub_;
    rclcpp::Subscription<MatchStateMsg>::SharedPtr match_state_sub_;
    rclcpp::Subscription<GoalieMsg>::SharedPtr goalie_sub_;
    rclcpp::Subscription<TeamColorMsg>::SharedPtr team_color_sub_;
    rclcpp::Subscription<TeamInfoMsg>::SharedPtr our_team_info_sub_;
    rclcpp::Subscription<TeamInfoMsg>::SharedPtr their_team_info_sub_;

    std::thread worker_;

    Context* context_;
};

}  // namespace ros2_temp
