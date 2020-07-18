#include "referee_sub.hpp"

#include <rj_constants/topic_names.hpp>

namespace ros2_temp {

static TeamInfo team_info_from_msg(const TeamInfoMsg& msg) {
    TeamInfo result;
    result.name = msg.name;
    result.score = msg.score;
    result.red_cards = msg.num_red_cards;
    result.yellow_cards = msg.num_yellow_cards;

    result.yellow_card_times.reserve(msg.num_yellow_cards);
    for (const auto& duration : msg.yellow_card_remaining_times) {
        result.yellow_card_times.emplace_back(RJ::FromROSDuration(duration));
    }

    result.timeouts_left = msg.timeouts_left;
    result.timeout_time = RJ::FromROSDuration(msg.remaining_timeout_time);
    result.goalie = msg.goalie_id;

    return result;
};

void RefereeSub::handle_game_state_msg(GameStateMsg::UniquePtr msg) {
    context_->game_state.period = static_cast<GameState::Period>(msg->period);
    context_->game_state.state = static_cast<GameState::State>(msg->state);
    context_->game_state.restart = static_cast<GameState::Restart>(msg->restart);
    context_->game_state.our_restart = msg->our_restart;
    context_->game_state.stage_time_end = RJ::FromROSTime(msg->stage_time_end);
}

RefereeSub::RefereeSub(Context* context, rclcpp::Executor* executor)
        : context_(context) {
    node_ = std::make_shared<rclcpp::Node>("_referee_receiver");
    executor->add_node(node_, true);

    raw_sub_ = node_->create_subscription<RawProtobufMsg>(
        referee::topics::kRefereeRawPub, rclcpp::QoS(10),
        [this] (RawProtobufMsg::UniquePtr msg) {
            context_->referee_packets.emplace_back();
            context_->referee_packets.back().ParseFromArray(msg->data.data(),
                                                            msg->data.size());
        });

    rclcpp::QoS keep_latest(1);
    keep_latest.keep_last(1);

    game_state_sub_ = node_->create_subscription<GameStateMsg>(
        referee::topics::kGameStatePub, keep_latest,
        [this] (GameStateMsg::UniquePtr msg) {
            context_->game_state.period = static_cast<GameState::Period>(msg->period);
            context_->game_state.state = static_cast<GameState::State>(msg->state);
            context_->game_state.restart = static_cast<GameState::Restart>(msg->restart);
            context_->game_state.our_restart = msg->our_restart;
            context_->game_state.stage_time_end = RJ::FromROSTime(msg->stage_time_end);
        });

    goalie_sub_ = node_->create_subscription<GoalieMsg>(
        referee::topics::kGoaliePub, keep_latest,
        [this] (GoalieMsg::UniquePtr msg) {
            // NOP, get goalie info in the TeamInfo message
        });

    team_color_sub_ = node_->create_subscription<TeamColorMsg>(
        referee::topics::kTeamColorPub, keep_latest,
        [this] (TeamColorMsg::UniquePtr msg) {
            context_->blue_team = msg->is_blue;
        });

    our_team_info_sub_ = node_->create_subscription<TeamInfoMsg>(
        referee::topics::kOurInfoPub, keep_latest,
        [this] (TeamInfoMsg::UniquePtr msg) {
            context_->our_info = team_info_from_msg(*msg);
        });

    their_team_info_sub_ = node_->create_subscription<TeamInfoMsg>(
        referee::topics::kTheirInfoPub, keep_latest,
        [this] (TeamInfoMsg::UniquePtr msg) {
          context_->their_info = team_info_from_msg(*msg);
        });
}

} // namespace ros2_temp
