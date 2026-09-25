#pragma once

#include <spdlog/spdlog.h>

#include <rj_common/field_dimensions.hpp>
#include <rj_common/referee_enums.hpp>
#include <rj_common/time.hpp>
#include <rj_constants/constants.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_geometry/geometry_conversions.hpp>
#include <rj_geometry/point.hpp>
#include <rj_geometry/transform_matrix.hpp>
#include <rj_msgs/msg/match_state.hpp>
#include <rj_msgs/msg/play_state.hpp>

struct PlayState {
public:
    using Msg = rj_msgs::msg::PlayState;

    PlayState() : PlayState(State::Halt, Restart::None, false, {}) {}

    enum State {
        Halt,           // Robots must not move
        Stop,           // Robots must stay 500mm away from ball + max speed of robots < 1.5 m/s
        Setup,          // Robots not on starting team must stay 500mm away from ball
        Ready,          // A robot on the starting team may kick the ball
        Playing,        // Normal play
        PenaltyPlaying  // Like normal play, but robots (except goalie and striker) should not do
                        // anything.
    };

    enum Restart { None, Kickoff, Free, Penalty, Placement };

    [[nodiscard]] State state() const { return state_; }
    [[nodiscard]] bool is_restart() const { return restart_ != Restart::None; }
    [[nodiscard]] Restart restart() const { return restart_; }
    [[nodiscard]] bool is_our_restart() const { return is_restart() && our_restart_; }
    [[nodiscard]] bool is_their_restart() const { return is_restart() && !our_restart_; }
    [[nodiscard]] bool is_halt() const { return state() == State::Halt; }
    [[nodiscard]] bool is_stop() const { return state() == State::Stop; }
    [[nodiscard]] bool is_setup() const { return state() == State::Setup; }
    [[nodiscard]] bool is_ready() const { return state() == State::Ready; }
    [[nodiscard]] bool is_playing() const { return state() == State::Playing; }
    [[nodiscard]] bool is_penalty_playing() const { return state() == State::PenaltyPlaying; }
    [[nodiscard]] bool is_kickoff() const { return restart() == Restart::Kickoff; }
    [[nodiscard]] bool is_free_kick() const { return restart() == Restart::Free; }
    [[nodiscard]] bool is_penalty() const { return restart() == Restart::Penalty; }
    [[nodiscard]] bool is_placement() const { return restart() == Restart::Placement; }

    [[nodiscard]] std::optional<rj_geometry::Point> ball_placement_point() const {
        if (is_placement()) {
            return ball_placement_point_;
        }
        return std::nullopt;
    }

    [[nodiscard]] bool wait_for_kick() const { return state_ == Ready; }

    void set_our_restart(bool our_restart) {
        if (is_restart()) {
            our_restart_ = our_restart;
        }
    }

    /**
     * @brief The ball was kicked; advance the state (from ready to playing) if necessary or else do
     * nothing.
     */
    [[nodiscard]] PlayState advanced_from_kick() const {
        if (wait_for_kick()) {
            if (restart_ == Restart::Penalty) {
                return PlayState::penalty_playing(our_restart_);
            }
            return PlayState::playing();
        }
        return *this;
    }

    /**
     * @brief We got a normal start command; if we're in setup advance to the corresponding restart.
     * nothing.
     */
    [[nodiscard]] PlayState advanced_from_normal_start() const {
        if (state_ == State::Setup) {
            if (restart_ == Restart::Kickoff) {
                return PlayState::ready_kickoff(our_restart_);
            }

            if (restart_ == Restart::Penalty) {
                return PlayState::ready_penalty(our_restart_);
            }

            SPDLOG_WARN(
                "Got a normal start command but restart is {} (normal start should only be "
                "necessary in kickoff/penalty)",
                restart_);
        }
        return *this;
    }

    /**
     * @brief Get the corresponding play state for the opposite team (restart team is flipped)
     */
    [[nodiscard]] PlayState opposite_team() const {
        return PlayState(state_, restart_, is_restart() && !our_restart_, ball_placement_point_);
    }

    /**
     * @brief Helper function to get the play state for the opposite team when the argument is true
     * and the same team when it is false. See @ref opposite_team
     */
    [[nodiscard]] PlayState opposite_team(bool do_invert) const {
        return do_invert ? opposite_team() : *this;
    }

    /**
     * @brief Get the corresponding play state with a transform applied to the ball placement point,
     * if it exists.
     */
    [[nodiscard]] PlayState with_transform(const rj_geometry::TransformMatrix& transform) const {
        return PlayState(state_, restart_, our_restart_,
                         is_placement() ? transform * ball_placement_point_ : rj_geometry::Point());
    }

    static PlayState halt() { return PlayState(State::Halt, Restart::None, false, {}); }
    static PlayState stop() { return PlayState(State::Stop, Restart::None, false, {}); }
    static PlayState setup_kickoff(bool ours) {
        return PlayState(State::Setup, Restart::Kickoff, ours, {});
    }
    static PlayState ready_kickoff(bool ours) {
        return PlayState(State::Ready, Restart::Kickoff, ours, {});
    }
    static PlayState setup_penalty(bool ours) {
        return PlayState(State::Setup, Restart::Penalty, ours, {});
    }
    static PlayState ready_penalty(bool ours) {
        return PlayState(State::Ready, Restart::Penalty, ours, {});
    }
    static PlayState ready_free_kick(bool ours) {
        return PlayState(State::Ready, Restart::Free, ours, {});
    }
    static PlayState ball_placement(bool ours, rj_geometry::Point point) {
        return PlayState(State::Stop, Restart::Placement, ours, point);
    }
    static PlayState playing() { return PlayState(State::Playing, Restart::None, false, {}); }
    static PlayState penalty_playing(bool ours) {
        return PlayState(State::PenaltyPlaying, Restart::None, ours, {});
    }

    bool operator==(const PlayState& other) const {
        return state() == other.state() && restart() == other.restart() &&
               is_our_restart() == other.is_our_restart() &&
               ball_placement_point() == other.ball_placement_point();
    }

    [[nodiscard]] bool same_as(const PlayState& other) const {
        return state_ == other.state_ && restart_ == other.restart_ &&
               our_restart_ == other.our_restart_;
    }

    bool operator!=(const PlayState& other) const { return !(*this == other); }

    std::string get_human_readout() const {
        std::string ours_string = is_our_restart() ? "OUR" : "THEIR";
        std::string setup_string = is_setup() ? "SETUP" : "READY";
        switch (restart_) {
            case Kickoff:
                return fmt::format("{} KICKOFF : {}", ours_string, setup_string);
            case Free:
                return fmt::format("{} FREE", ours_string);
            case Penalty:
                return fmt::format("{} PENALTY : {}", ours_string, setup_string);
            case Placement:
                return fmt::format("{} PLACEMENT({:.2f}, {:.2f})", ours_string,
                                   ball_placement_point_.x(), ball_placement_point_.y());
            default:
            case None:
                break;
        }

        switch (state_) {
            case Halt:
                return "HALT";
            case Stop:
                return "STOP";
            case Playing:
                return "PLAYING";
            case PenaltyPlaying:
                return "PENALTY_PLAY";
            default:
            case Setup:
            case Ready:
                // Should have been covered by the restart case!
                break;
        }

        SPDLOG_WARN("Trying to convert invalid game state to string: state is {}, restart is {}",
                    state_, restart_);
        return "???";
    }

private:
    friend struct rclcpp::TypeAdapter<PlayState, rj_msgs::msg::PlayState>;

    PlayState(State state, Restart restart, bool our_restart,
              rj_geometry::Point ball_placement_point)
        : state_{state},
          restart_{restart},
          our_restart_{our_restart},
          ball_placement_point_{ball_placement_point} {}

    State state_;
    Restart restart_;
    bool our_restart_;
    rj_geometry::Point ball_placement_point_;
};

struct MatchState {
public:
    using Msg = rj_msgs::msg::MatchState;

    enum Period { FirstHalf, Halftime, SecondHalf, Overtime1, Overtime2, PenaltyShootout };

    Period period;

    // Time in seconds remaining in the current period
    RJ::Seconds stage_time_left{};

    std::optional<rj_geometry::Point> ball_placement_point;
};

namespace rclcpp {

template <>
struct TypeAdapter<MatchState::Period, uint8_t> {
    using is_specialized = std::true_type;
    using custom_type = MatchState::Period;
    using ros_message_type = uint8_t;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination = static_cast<uint8_t>(source);
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = static_cast<MatchState::Period>(source);
    }
};

template <>
struct TypeAdapter<MatchState, rj_msgs::msg::MatchState> {
    using is_specialized = std::true_type;
    using custom_type = MatchState;
    using ros_message_type = rj_msgs::msg::MatchState;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        rj_convert::convert_to_ros(source.period, &destination.period);
        rj_convert::convert_to_ros(source.stage_time_left, &destination.stage_time_left);
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        rj_convert::convert_from_ros(source.period, &destination.period);
        rj_convert::convert_from_ros(source.stage_time_left, &destination.stage_time_left);
    }
};


template <>
struct TypeAdapter<PlayState::State, uint8_t> {
    using is_specialized = std::true_type;
    using custom_type = PlayState::State;
    using ros_message_type = uint8_t;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination = static_cast<uint8_t>(source);
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = static_cast<PlayState::State>(source);
    }
};

template <>
struct TypeAdapter<PlayState::Restart, uint8_t> {
    using is_specialized = std::true_type;
    using custom_type = PlayState::Restart;
    using ros_message_type = uint8_t;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        destination = static_cast<uint8_t>(source);
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        destination = static_cast<PlayState::Restart>(source);
    }
};

template <>
struct TypeAdapter<PlayState, rj_msgs::msg::PlayState> {
    using is_specialized = std::true_type;
    using custom_type = PlayState;
    using ros_message_type = rj_msgs::msg::PlayState;

    static void convert_to_ros(const custom_type& source, ros_message_type& destination) {
        rj_convert::convert_to_ros(source.state(), &destination.state);
        rj_convert::convert_to_ros(source.restart(), &destination.restart);
        rj_convert::convert_to_ros(source.is_our_restart(), &destination.our_restart);
        if (source.ball_placement_point().has_value()) {
            rj_convert::convert_to_ros(source.ball_placement_point().value(),
                                       &destination.placement_point);
        }
    }

    static void convert_to_custom(const ros_message_type& source, custom_type& destination) {
        PlayState::State state = PlayState::State::Halt;
        rj_convert::convert_from_ros(source.state, &state);
        PlayState::Restart restart = PlayState::Restart::None;
        rj_convert::convert_from_ros(source.restart, &restart);
        destination = PlayState(
            state, restart, source.our_restart,
            rj_convert::convert_from_ros<rj_geometry_msgs::msg::Point, rj_geometry::Point>(
                source.placement_point));
    }
};


}  // namespace rclcpp
