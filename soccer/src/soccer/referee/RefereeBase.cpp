#include "RefereeBase.hpp"

#include <rj_common/Utils.hpp>
#include <rj_constants/topic_names.hpp>

namespace referee {

RefereeBase::RefereeBase(const std::string& name)
    : rclcpp::Node(name), _param_provider(this, kRefereeParamModule) {
    auto keep_latest = rclcpp::QoS(1).transient_local();

    _team_color_pub = create_publisher<TeamColorMsg>(
        referee::topics::kTeamColorPub, keep_latest);
    _goalie_id_pub =
        create_publisher<GoalieMsg>(referee::topics::kGoaliePub, keep_latest);
    _our_team_info_pub = create_publisher<TeamInfoMsg>(
        referee::topics::kOurInfoPub, keep_latest);
    _their_team_info_pub = create_publisher<TeamInfoMsg>(
        referee::topics::kTheirInfoPub, keep_latest);
    _game_state_pub = create_publisher<GameStateMsg>(
        referee::topics::kGameStatePub, keep_latest);
}

void RefereeBase::play() {
    update_cache(_state.state, GameState::State::Playing, &_state_valid);
    update_cache(_state.restart, GameState::Restart::None, &_state_valid);
}

void RefereeBase::stop() {
    update_cache(_state.state, GameState::State::Stop, &_state_valid);
    update_cache(_state.restart, GameState::Restart::None, &_state_valid);
}

void RefereeBase::halt() {
    // The restart carries through halts, so there is no need to change the
    // restart state.
    update_cache(_state.state, GameState::State::Halt, &_state_valid);
}

void RefereeBase::setup() {
    update_cache(_state.state, GameState::State::Setup, &_state_valid);
}

void RefereeBase::ready() {
    update_cache(_state.state, GameState::State::Ready, &_state_valid);
}

void RefereeBase::restart(GameState::Restart type, bool blue_restart) {
    update_cache(_state.restart, type, &_state_valid);
    update_cache(_blue_restart, blue_restart, &_state_valid);
    update_cache(_state.our_restart, _blue_team == _blue_restart,
                 &_state_valid);
    if (_state.restart != GameState::Restart::Placement) {
        _state.ball_placement_point = std::nullopt;
    }
}

void RefereeBase::ball_placement(Geometry2d::Point point, bool blue_placement) {
    restart(GameState::Restart::Placement, blue_placement);
    update_cache(_state.ball_placement_point, std::make_optional(point),
                 &_state_valid);
}

void RefereeBase::set_period(GameState::Period period) {
    update_cache(_state.period, period, &_state_valid);
}

void RefereeBase::set_stage_time_left(RJ::Seconds stage_time_left) {
    update_cache(_state.stage_time_left, stage_time_left, &_state_valid);
}

void RefereeBase::set_team_name(const std::string& name) {
    _our_name = name;
    update_team_color_from_names();
}

void RefereeBase::set_team_info(const TeamInfo& blue, const TeamInfo& yellow) {
    update_cache(_blue_info, blue, &_team_info_valid);
    update_cache(_yellow_info, yellow, &_team_info_valid);

    update_team_color_from_names();

    _goalie_valid &= _blue_team ? blue.goalie == _blue_info.goalie
                                : yellow.goalie == _yellow_info.goalie;
}

void RefereeBase::set_team_color(bool is_blue) {
    bool valid = true;
    update_cache(_blue_team, is_blue, &valid);

    // All information except for game state needs to change.
    // In game state the only field that would need to change is our_restart,
    // which should be calculated from blue_restart anyway.
    _goalie_valid &= valid;
    _team_info_valid &= valid;
    _goalie_valid &= valid;

    update_cache(_state.our_restart, _blue_team == _blue_restart,
                 &_state_valid);
}

void RefereeBase::set_goalie(uint8_t goalie_id) {
    bool valid = true;
    update_cache(_blue_team ? _blue_info.goalie : _yellow_info.goalie,
                 static_cast<uint>(goalie_id), &valid);
    _team_info_valid &= valid;
    _goalie_valid &= valid;
}

void RefereeBase::send() {
    if (!_goalie_valid) {
        GoalieMsg msg;
        msg.goalie_id = _blue_team ? _blue_info.goalie : _yellow_info.goalie;
        _goalie_id_pub->publish(msg);
        _goalie_valid = true;
    }

    if (!_state_valid) {
        GameState::Msg msg;
        rj_convert::convert_to_ros(_state, &msg);
        _game_state_pub->publish(msg);
        _state_valid = true;
    }

    if (!_team_info_valid) {
        auto our_msg = _blue_team ? _blue_info : _yellow_info;
        auto their_msg = _blue_team ? _yellow_info : _blue_info;
        _our_team_info_pub->publish(
            rj_convert::convert_to_ros<TeamInfo>(our_msg));
        _their_team_info_pub->publish(
            rj_convert::convert_to_ros<TeamInfo>(our_msg));
        _team_info_valid = true;
    }

    if (!_blue_team_valid) {
        TeamColorMsg team_color;
        team_color.is_blue = _blue_team;
        _team_color_pub->publish(team_color);
        _blue_team_valid = true;
    }
}

void RefereeBase::update_team_color_from_names() {
    if (_our_name.empty() || _blue_info.name == _yellow_info.name) {
        return;
    }

    if (_our_name == _blue_info.name) {
        set_team_color(true);
    } else if (_our_name == _yellow_info.name) {
        set_team_color(false);
    }
}

}  // namespace referee