#pragma once

#include <rclcpp/executor.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rj_common/context.hpp>
#include <rj_common/game_state.hpp>
#include <rj_common/node.hpp>
#include <rj_common/topic_names.hpp>
#include <rj_msgs/msg/goalie.hpp>
#include <rj_msgs/msg/raw_protobuf.hpp>
#include <rj_msgs/msg/team_color.hpp>
#include <rj_utils/logging_macros.hpp>

namespace ros2_temp {

using RawProtobufMsg = rj_msgs::msg::RawProtobuf;
using GoalieMsg = rj_msgs::msg::Goalie;
using TeamColorMsg = rj_msgs::msg::TeamColor;

class RefereeSub : public Node {
public:
    RefereeSub(Context* context, rclcpp::Executor* executor);

    void run() override {}

private:
    std::shared_ptr<rclcpp::Node> node_;

    rclcpp::Subscription<RawProtobufMsg>::SharedPtr raw_sub_;
    rclcpp::Subscription<PlayState>::SharedPtr play_state_sub_;
    rclcpp::Subscription<MatchState>::SharedPtr match_state_sub_;
    rclcpp::Subscription<GoalieMsg>::SharedPtr goalie_sub_;
    rclcpp::Subscription<TeamColorMsg>::SharedPtr team_color_sub_;
    rclcpp::Subscription<TeamInfo>::SharedPtr our_team_info_sub_;
    rclcpp::Subscription<TeamInfo>::SharedPtr their_team_info_sub_;

    std::thread worker_;

    Context* context_;
};

}  // namespace ros2_temp
