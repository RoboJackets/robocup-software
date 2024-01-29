#include "referee_base.hpp"

#include <rj_common/transforms.hpp>
#include <rj_common/utils.hpp>
#include <rj_constants/topic_names.hpp>

#include "game_state.hpp"

namespace referee {

RefereeBase::RefereeBase(const std::string& name)
    : Node{name, rclcpp::NodeOptions{}
                     .automatically_declare_parameters_from_overrides(true)
                     .allow_undeclared_parameters(true)},
      param_provider_(this, kRefereeParamModule),
      config_client_{this} {
    auto keep_latest = rclcpp::QoS(1).transient_local();

    team_color_pub_ = create_publisher<TeamColorMsg>(referee::topics::kTeamColorTopic, keep_latest);
    goalie_id_pub_ = create_publisher<GoalieMsg>(referee::topics::kGoalieTopic, keep_latest);
    our_team_info_pub_ = create_publisher<TeamInfoMsg>(referee::topics::kOurInfoTopic, keep_latest);
    their_team_info_pub_ =
        create_publisher<TeamInfoMsg>(referee::topics::kTheirInfoTopic, keep_latest);
    play_state_pub_ =
        create_publisher<PlayState::Msg>(referee::topics::kPlayStateTopic, keep_latest);
    match_state_pub_ =
        create_publisher<MatchState::Msg>(referee::topics::kMatchStateTopic, keep_latest);

    pub_timer_ = create_wall_timer(100ms, [this]() { send(); });

    world_state_sub_ = create_subscription<WorldState::Msg>(
        vision_filter::topics::kWorldStateTopic, 1, [this](WorldState::Msg::SharedPtr msg) {
            auto ball_state = rj_convert::convert_from_ros(msg->ball);
            if (spin_kick_detector(ball_state.position)) {
                send();
            }
        });
}

void RefereeBase::set_period(MatchState::Period period) { match_state_.period = period; }

void RefereeBase::set_stage_time_left(RJ::Seconds stage_time_left) {
    match_state_.stage_time_left = stage_time_left;
}

void RefereeBase::set_team_name(const std::string& name) {
    our_name_ = name;

    update_team_color_from_names();
}

void RefereeBase::set_team_info(const TeamInfo& blue, const TeamInfo& yellow) {
    blue_info_ = blue;
    yellow_info_ = yellow;

    update_team_color_from_names();
}

void RefereeBase::set_team_color(bool is_blue) {
    has_any_info_ = true;
    blue_team_ = is_blue;
}

void RefereeBase::override_goalie(uint8_t goalie_id) {
    if (blue_team_) {
        blue_info_.goalie = goalie_id;
    } else {
        yellow_info_.goalie = goalie_id;
    }
}

static PlayState resolve_play_state(const PlayState& yellow_vision_centric_play_state,
                                    bool blue_team, bool defend_plus_x,
                                    const FieldDimensions& dimensions) {
    return yellow_vision_centric_play_state.opposite_team(blue_team).with_transform(
        rj_common::world_to_team(dimensions, defend_plus_x));
}

void RefereeBase::send() {
    if (!has_any_info_ || !config_client_.connected()) {
        return;
    }

    GoalieMsg goalie_msg;
    goalie_msg.goalie_id = blue_team_ ? blue_info_.goalie : yellow_info_.goalie;
    goalie_id_pub_->publish(goalie_msg);

    const auto resolved_play_state = resolve_play_state(
        yellow_play_state_, blue_team_, config_client_.game_settings().defend_plus_x,
        rj_convert::convert_from_ros(config_client_.field_dimensions()));
    play_state_pub_->publish(rj_convert::convert_to_ros(resolved_play_state));
    match_state_pub_->publish(rj_convert::convert_to_ros(match_state_));

    auto our_info = blue_team_ ? blue_info_ : yellow_info_;
    auto their_info = blue_team_ ? yellow_info_ : blue_info_;
    our_team_info_pub_->publish(rj_convert::convert_to_ros(our_info));
    their_team_info_pub_->publish(rj_convert::convert_to_ros(their_info));

    TeamColorMsg team_color;
    team_color.is_blue = blue_team_;
    team_color_pub_->publish(team_color);
}

void RefereeBase::update_team_color_from_names() {
    if (our_name_.empty() || blue_info_.name == yellow_info_.name) {
        return;
    }

    if (our_name_ == blue_info_.name) {
        set_team_color(true);
    } else if (our_name_ == yellow_info_.name) {
        set_team_color(false);
    }
}

}  // namespace referee
