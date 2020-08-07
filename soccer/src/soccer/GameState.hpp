#pragma once

#include <Geometry2d/GeometryConversions.hpp>
#include <Geometry2d/Point.hpp>
#include <Geometry2d/TransformMatrix.hpp>
#include <rj_common/Field_Dimensions.hpp>
#include <rj_common/RefereeEnums.hpp>
#include <rj_common/time.hpp>
#include <rj_constants/constants.hpp>
#include <rj_convert/ros_convert.hpp>
#include <rj_msgs/msg/game_state.hpp>

/**
 * @brief Holds the state of the game according to the referee
 *
 * @details Contains information on what period of the game it is, what type of
 * play to run, team scores, time remaining, etc.  During normal gameplay, the
 * information in this class is received from the [ssl
 * refbox](https://github.com/RoboCup-SSL/ssl-refbox)
 * program over the network.
 */
class GameState {
public:
    using Msg = rj_msgs::msg::GameState;

    enum Period {
        FirstHalf,
        Halftime,
        SecondHalf,
        Overtime1,
        Overtime2,
        PenaltyShootout
    };

    enum State {
        Halt,    // Robots must not move
        Stop,    // Robots must stay 500mm away from ball
        Setup,   // Robots not on starting team must stay 500mm away from ball
        Ready,   // A robot on the starting team may kick the ball
        Playing  // Normal play
    };

    // Types of restarts
    enum Restart { None, Kickoff, Direct, Indirect, Penalty, Placement };

    Period period = Period::FirstHalf;
    State state = State::Halt;
    Restart restart = Restart::None;

    // True if our team can kick the ball during a restart
    bool our_restart = false;

    // Time in seconds remaining in the current period
    RJ::Seconds stage_time_left{};

    std::optional<Geometry2d::Point> ball_placement_point;

    GameState() = default;

    GameState(
        Period period, State state, Restart restart, bool our_restart,
        RJ::Seconds stage_time_left,
        std::optional<Geometry2d::Point> ball_placement_point = std::nullopt)
        : period{period},
          state{state},
          restart{restart},
          our_restart{our_restart},
          stage_time_left{stage_time_left},
          ball_placement_point{ball_placement_point} {}

    ////////
    // Rule queries

    [[nodiscard]] bool is_first_half() const { return period == FirstHalf; }

    [[nodiscard]] bool is_second_half() const { return period == SecondHalf; }

    [[nodiscard]] bool is_halftime() const { return period == Halftime; }

    [[nodiscard]] bool is_overtime1() const { return period == Overtime1; }

    [[nodiscard]] bool is_overtime2() const { return period == Overtime2; }

    [[nodiscard]] bool is_penalty_shootout() const {
        return period == PenaltyShootout;
    }

    [[nodiscard]] bool halt() const { return state == Halt; }

    [[nodiscard]] bool stopped() const { return state == Stop; }

    [[nodiscard]] bool playing() const { return state == Playing; }

    [[nodiscard]] bool kickoff() const { return restart == Kickoff; }

    [[nodiscard]] bool penalty() const { return restart == Penalty; }

    [[nodiscard]] bool placement() const { return restart == Placement; }

    [[nodiscard]] bool is_our_restart() const { return our_restart; }

    [[nodiscard]] bool direct() const { return restart == Direct; }

    [[nodiscard]] bool indirect() const { return restart == Indirect; }

    [[nodiscard]] bool our_kickoff() const { return kickoff() && our_restart; }

    [[nodiscard]] bool our_penalty() const { return penalty() && our_restart; }

    [[nodiscard]] bool our_direct() const { return direct() && our_restart; }

    [[nodiscard]] bool our_indirect() const { return indirect() && our_restart; }

    [[nodiscard]] bool our_free_kick() const {
        return our_direct() || our_indirect();
    }

    [[nodiscard]] bool our_placement() const {
        return placement() && our_restart;
    }

    [[nodiscard]] bool their_kickoff() const {
        return kickoff() && !our_restart;
    }

    [[nodiscard]] bool their_penalty() const {
        return penalty() && !our_restart;
    }

    [[nodiscard]] bool their_direct() const { return direct() && !our_restart; }

    [[nodiscard]] bool their_indirect() const {
        return indirect() && !our_restart;
    }

    [[nodiscard]] bool their_free_kick() const {
        return their_direct() || their_indirect();
    }

    [[nodiscard]] bool their_placement() const {
        return placement() && !our_restart;
    }

    // Robots must be in position for a restart
    [[nodiscard]] bool setup_restart() const {
        return state == Setup || state == Ready;
    }

    [[nodiscard]] bool in_setup_state() const { return state == Setup; }

    [[nodiscard]] bool in_ready_state() const { return state == Ready; }

    // One of our robots can kick the ball
    [[nodiscard]] bool can_kick() const {
        return state == Playing || (our_restart && state == Playing);
    }

    // Our robots must stay 500mm away from the ball
    [[nodiscard]] bool stay_away_from_ball() const {
        return state != Playing && !our_restart;
    }

    // Our robots must stay on our half of the field
    [[nodiscard]] bool stay_on_side() const {
        return setup_restart() && restart == Kickoff;
    }

    // Our robots (except the penalty kicker) must stay 400mm behind the penalty
    // line
    [[nodiscard]] bool stay_behind_penalty_line() const {
        return restart == Penalty;
    }

    [[nodiscard]] std::optional<Geometry2d::Point> get_ball_placement_point()
        const {
        return ball_placement_point;
    }
};

namespace rj_convert {

template <>
struct RosConverter<GameState::Period, uint8_t> {
    static uint8_t to_ros(const GameState::Period& from) {
        return static_cast<uint8_t>(from);
    }

    static GameState::Period from_ros(const uint8_t from) {
        return static_cast<GameState::Period>(from);
    }
};

template <>
struct RosConverter<GameState::State, uint8_t> {
    static uint8_t to_ros(const GameState::State& from) {
        return static_cast<uint8_t>(from);
    }

    static GameState::State from_ros(const uint8_t from) {
        return static_cast<GameState::State>(from);
    }
};

template <>
struct RosConverter<GameState::Restart, uint8_t> {
    static uint8_t to_ros(const GameState::Restart& from) {
        return static_cast<uint8_t>(from);
    }

    static GameState::Restart from_ros(const uint8_t from) {
        return static_cast<GameState::Restart>(from);
    }
};

template <>
struct RosConverter<GameState, rj_msgs::msg::GameState> {
    static rj_msgs::msg::GameState to_ros(const GameState& from) {
        rj_msgs::msg::GameState to;
        convert_to_ros(from.period, &to.period);
        convert_to_ros(from.state, &to.state);
        convert_to_ros(from.restart, &to.restart);
        convert_to_ros(from.our_restart, &to.our_restart);
        convert_to_ros(from.stage_time_left, &to.stage_time_left);
        if (from.ball_placement_point.has_value()) {
            convert_to_ros(from.ball_placement_point.value(),
                           &to.placement_point);
        }
        return to;
    }

    static GameState from_ros(const rj_msgs::msg::GameState& from) {
        GameState to;
        convert_from_ros(from.period, &to.period);
        convert_from_ros(from.state, &to.state);
        convert_from_ros(from.restart, &to.restart);
        convert_from_ros(from.our_restart, &to.our_restart);
        convert_from_ros(from.stage_time_left, &to.stage_time_left);
        if (to.restart == GameState::Restart::Placement) {
            to.ball_placement_point = convert_from_ros(from.placement_point);
        }
        return to;
    }
};

ASSOCIATE_CPP_ROS(GameState, GameState::Msg);

}  // namespace rj_convert