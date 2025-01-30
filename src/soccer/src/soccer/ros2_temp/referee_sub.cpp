#include "referee_sub.hpp"

#include <rj_constants/topic_names.hpp>
#include <rj_utils/logging_macros.hpp>

#include "game_state.hpp"

namespace ros2_temp {

RefereeSub::RefereeSub(Context* context, rclcpp::Executor* executor) : context_(context) {
    node_ = std::make_shared<rclcpp::Node>("_referee_receiver");
    executor->add_node(node_, true);

    raw_sub_ = node_->create_subscription<RawProtobufMsg>(
        referee::topics::kRefereeRawTopic, rclcpp::QoS(10), [this](RawProtobufMsg::UniquePtr msg) {
            context_->referee_packets.emplace_back();
            context_->referee_packets.back().ParseFromArray(msg->data.data(), msg->data.size());
        });

    auto keep_latest = rclcpp::QoS(1).transient_local();

    play_state_sub_ = node_->create_subscription<PlayState::Msg>(
        referee::topics::kPlayStateTopic, keep_latest, [this](PlayState::Msg::UniquePtr msg) {
            rj_convert::convert_from_ros(*msg, &context_->play_state);
        });

    match_state_sub_ = node_->create_subscription<MatchStateMsg>(
        referee::topics::kMatchStateTopic, keep_latest, [this](MatchStateMsg::UniquePtr msg) {
            rj_convert::convert_from_ros(*msg, &context_->match_state);
        });

    team_color_sub_ = node_->create_subscription<TeamColorMsg>(
        referee::topics::kTeamColorTopic, keep_latest, [this](TeamColorMsg::UniquePtr msg) {
            rj_convert::convert_from_ros(msg->is_blue, &context_->blue_team);
        });

    goalie_sub_ = node_->create_subscription<GoalieMsg>(
        referee::topics::kGoalieTopic, keep_latest, []([[maybe_unused]] GoalieMsg::UniquePtr msg) {
            // NOP, get goalie info in the TeamInfo message
        });

    our_team_info_sub_ = node_->create_subscription<TeamInfoMsg>(
        referee::topics::kOurInfoTopic, keep_latest, [this](TeamInfoMsg::UniquePtr msg) {
            rj_convert::convert_from_ros(*msg, &context_->our_info);
        });

    their_team_info_sub_ = node_->create_subscription<TeamInfoMsg>(
        referee::topics::kTheirInfoTopic, keep_latest, [this](TeamInfoMsg::UniquePtr msg) {
            rj_convert::convert_from_ros(*msg, &context_->their_info);
        });
}

}  // namespace ros2_temp
