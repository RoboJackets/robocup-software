#include "referee_base.hpp"

#include <rj_common/time.hpp>
#include <rj_common/utils.hpp>
#include <rj_constants/topic_names.hpp>

namespace referee {

DEFINE_FLOAT64(kRefereeParamModule, kick_threshold, kBallRadius * 3,
               "Distance in meters that ball must travel to detect kick")

DEFINE_FLOAT64(
    kRefereeParamModule, kick_verify_time, 0.250,
    "How long in seconds ball must be more than kick_threshold meters away to detect kick")

RefereeBase::RefereeBase(const std::string& name) : rclcpp::Node(name) {
    auto keep_latest = rclcpp::QoS(1).transient_local();

    team_color_pub_ = create_publisher<TeamColorMsg>(referee::topics::kTeamColorPub, keep_latest);
    goalie_id_pub_ = create_publisher<GoalieMsg>(referee::topics::kGoaliePub, keep_latest);
    our_team_info_pub_ = create_publisher<TeamInfoMsg>(referee::topics::kOurInfoPub, keep_latest);
    their_team_info_pub_ =
        create_publisher<TeamInfoMsg>(referee::topics::kTheirInfoPub, keep_latest);
    game_state_pub_ = create_publisher<GameStateMsg>(referee::topics::kGameStatePub, keep_latest);

    auto callback = [this](WorldState::Msg::SharedPtr world_state) {  // NOLINT
        ball_state_ = rj_convert::convert_from_ros(world_state->ball);
    };
    world_state_sub_ = create_subscription<WorldState::Msg>(vision_filter::topics::kWorldStatePub,
                                                            rclcpp::QoS(1), callback);
}

void RefereeBase::play() {
    update_cache(state_.state, GameState::State::Playing, &state_valid_);
    update_cache(state_.restart, GameState::Restart::None, &state_valid_);
}

void RefereeBase::stop() {
    update_cache(state_.state, GameState::State::Stop, &state_valid_);
    update_cache(state_.restart, GameState::Restart::None, &state_valid_);
    kick_detect_state_ = KickDetectState::kCapturePosition;
}

void RefereeBase::halt() {
    // The restart carries through halts, so there is no need to change the
    // restart state.
    update_cache(state_.state, GameState::State::Halt, &state_valid_);
}

void RefereeBase::setup() {
    update_cache(state_.state, GameState::State::Setup, &state_valid_);
    kick_detect_state_ = KickDetectState::kCapturePosition;
}

void RefereeBase::ready() {
    update_cache(state_.state, GameState::State::Ready, &state_valid_);
    kick_detect_state_ = KickDetectState::kCapturePosition;
}

void RefereeBase::restart(GameState::Restart type, bool blue_restart) {
    update_cache(state_.restart, type, &state_valid_);
    update_cache(blue_restart_, blue_restart, &state_valid_);
    update_cache(state_.our_restart, blue_team_ == blue_restart_, &state_valid_);
    if (state_.restart != GameState::Restart::Placement) {
        state_.ball_placement_point = std::nullopt;
    }
}

void RefereeBase::ball_placement(rj_geometry::Point point, bool blue_placement) {
    restart(GameState::Restart::Placement, blue_placement);
    update_cache(state_.ball_placement_point, std::make_optional(point), &state_valid_);
}

void RefereeBase::set_period(GameState::Period period) {
    update_cache(state_.period, period, &state_valid_);
}

void RefereeBase::set_stage_time_left(RJ::Seconds stage_time_left) {
    update_cache(state_.stage_time_left, stage_time_left, &state_valid_);
}

void RefereeBase::set_team_name(const std::string& name) {
    our_name_ = name;
    update_team_color_from_names();
}

void RefereeBase::set_team_info(const TeamInfo& blue, const TeamInfo& yellow) {
    update_cache(blue_info_, blue, &team_info_valid_);
    update_cache(yellow_info_, yellow, &team_info_valid_);

    update_team_color_from_names();

    goalie_valid_ &=
        blue_team_ ? blue.goalie == blue_info_.goalie : yellow.goalie == yellow_info_.goalie;
}

void RefereeBase::set_team_color(bool is_blue) {
    has_any_info_ = true;

    bool valid = true;
    update_cache(blue_team_, is_blue, &valid);

    // All information except for game state needs to change.
    // In game state the only field that would need to change is our_restart,
    // which should be calculated from blue_restart anyway.
    goalie_valid_ &= valid;
    team_info_valid_ &= valid;
    goalie_valid_ &= valid;

    update_cache(state_.our_restart, blue_team_ == blue_restart_, &state_valid_);
}

void RefereeBase::set_goalie(uint8_t goalie_id) {
    bool valid = true;
    update_cache(blue_team_ ? blue_info_.goalie : yellow_info_.goalie, static_cast<uint>(goalie_id),
                 &valid);
    team_info_valid_ &= valid;
    goalie_valid_ &= valid;
}

void RefereeBase::spin_kick_detector() {
    // Only run kick detector when ball is visible
    if (!ball_state_.visible) {
        return;
    }

    switch (kick_detect_state_) {
        case KickDetectState::kCapturePosition:
            capture_ready_point_ = ball_state_.position;
            kick_detect_state_ = KickDetectState::kWaitForKick;
            break;
        case KickDetectState::kWaitForKick:
            if (ball_state_.position.near_point(capture_ready_point_, PARAM_kick_threshold)) {
                kick_time_ = RJ::now();
                kick_detect_state_ = KickDetectState::kVerifyKick;
            }
            break;
        case KickDetectState::kVerifyKick: {
            const RJ::Seconds elapsed_time = RJ::now() - kick_time_;

            if (ball_state_.position.near_point(capture_ready_point_, PARAM_kick_threshold)) {
                kick_detect_state_ = KickDetectState::kWaitForKick;
            } else if (elapsed_time.count() >= PARAM_kick_verify_time) {
                play();
                kick_detect_state_ = KickDetectState::kStandBy;
            }
            break;
        }
        case KickDetectState::kStandBy:
            break;
    }
}

void RefereeBase::send() {
    if (!has_any_info_) {
        return;
    }

    if (!goalie_valid_) {
        GoalieMsg msg;
        msg.goalie_id = blue_team_ ? blue_info_.goalie : yellow_info_.goalie;
        goalie_id_pub_->publish(msg);
        goalie_valid_ = true;
    }

    if (!state_valid_) {
        GameState::Msg msg;
        rj_convert::convert_to_ros(state_, &msg);
        game_state_pub_->publish(msg);
        state_valid_ = true;
    }

    if (!team_info_valid_) {
        auto our_msg = blue_team_ ? blue_info_ : yellow_info_;
        auto their_msg = blue_team_ ? yellow_info_ : blue_info_;
        our_team_info_pub_->publish(rj_convert::convert_to_ros(our_msg));
        their_team_info_pub_->publish(rj_convert::convert_to_ros(our_msg));
        team_info_valid_ = true;
    }

    if (!blue_team_valid_) {
        TeamColorMsg team_color;
        team_color.is_blue = blue_team_;
        team_color_pub_->publish(team_color);
        blue_team_valid_ = true;
    }
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
