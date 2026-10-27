#include "rj_ui/ros2_temp/referee_sub.hpp"

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

    play_state_sub_ = node_->create_subscription<PlayState>(
        referee::topics::kPlayStateTopic, keep_latest,
        [this](PlayState::ConstSharedPtr msg) { context_->play_state = *msg; });

    match_state_sub_ = node_->create_subscription<MatchState>(
        referee::topics::kMatchStateTopic, keep_latest,
        [this](MatchState::ConstSharedPtr msg) { context_->match_state = *msg; });

    team_color_sub_ = node_->create_subscription<TeamColorMsg>(
        referee::topics::kTeamColorTopic, keep_latest, [this](TeamColorMsg::UniquePtr msg) {
            context_->blue_team = msg->is_blue;
        });

    goalie_sub_ = node_->create_subscription<GoalieMsg>(
        referee::topics::kGoalieTopic, keep_latest, []([[maybe_unused]] GoalieMsg::UniquePtr msg) {
            // NOP, get goalie info in the TeamInfo message
        });

    our_team_info_sub_ = node_->create_subscription<TeamInfo>(
        referee::topics::kOurInfoTopic, keep_latest,
        [this](TeamInfo::ConstSharedPtr msg) { context_->our_info = *msg; });

    their_team_info_sub_ = node_->create_subscription<TeamInfo>(
        referee::topics::kTheirInfoTopic, keep_latest,
        [this](TeamInfo::ConstSharedPtr msg) { context_->their_info = *msg; });
}

}  // namespace ros2_temp
