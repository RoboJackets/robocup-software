#pragma once

#include <Geometry2d/Point.hpp>
#include <Geometry2d/TransformMatrix.hpp>
#include <rj_common/Field_Dimensions.hpp>
#include <rj_common/RefereeEnums.hpp>
#include <rj_common/time.hpp>
#include <rj_constants/constants.hpp>

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
    RJ::Seconds stage_time_left;

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

    [[nodiscard]] bool isFirstHalf() const { return period == FirstHalf; }

    [[nodiscard]] bool isSecondHalf() const { return period == SecondHalf; }

    [[nodiscard]] bool isHalftime() const { return period == Halftime; }

    [[nodiscard]] bool isOvertime1() const { return period == Overtime1; }

    [[nodiscard]] bool isOvertime2() const { return period == Overtime2; }

    [[nodiscard]] bool isPenaltyShootout() const { return period == PenaltyShootout; }

    [[nodiscard]] bool halt() const { return state == Halt; }

    [[nodiscard]] bool stopped() const { return state == Stop; }

    [[nodiscard]] bool playing() const { return state == Playing; }

    [[nodiscard]] bool kickoff() const { return restart == Kickoff; }

    [[nodiscard]] bool penalty() const { return restart == Penalty; }

    [[nodiscard]] bool placement() const { return restart == Placement; }

    [[nodiscard]] bool isOurRestart() const { return our_restart; }

    [[nodiscard]] bool direct() const { return restart == Direct; }

    [[nodiscard]] bool indirect() const { return restart == Indirect; }

    [[nodiscard]] bool ourKickoff() const { return kickoff() && our_restart; }

    [[nodiscard]] bool ourPenalty() const { return penalty() && our_restart; }

    [[nodiscard]] bool ourDirect() const { return direct() && our_restart; }

    [[nodiscard]] bool ourIndirect() const { return indirect() && our_restart; }

    [[nodiscard]] bool ourFreeKick() const { return ourDirect() || ourIndirect(); }

    [[nodiscard]] bool ourPlacement() const { return placement() && our_restart; }

    [[nodiscard]] bool theirKickoff() const { return kickoff() && !our_restart; }

    [[nodiscard]] bool theirPenalty() const { return penalty() && !our_restart; }

    [[nodiscard]] bool theirDirect() const { return direct() && !our_restart; }

    [[nodiscard]] bool theirIndirect() const { return indirect() && !our_restart; }

    [[nodiscard]] bool theirFreeKick() const { return theirDirect() || theirIndirect(); }

    [[nodiscard]] bool theirPlacement() const { return placement() && !our_restart; }

    // Robots must be in position for a restart
    [[nodiscard]] bool setupRestart() const { return state == Setup || state == Ready; }

    [[nodiscard]] bool inSetupState() const { return state == Setup; }

    [[nodiscard]] bool inReadyState() const { return state == Ready; }

    // One of our robots can kick the ball
    [[nodiscard]] bool canKick() const {
        return state == Playing || (our_restart && state == Playing);
    }

    // Our robots must stay 500mm away from the ball
    [[nodiscard]] bool stayAwayFromBall() const { return state != Playing && !our_restart; }

    // Our robots must stay on our half of the field
    [[nodiscard]] bool stayOnSide() const { return setupRestart() && restart == Kickoff; }

    // Our robots (except the penalty kicker) must stay 400mm behind the penalty
    // line
    [[nodiscard]] bool stayBehindPenaltyLine() const { return restart == Penalty; }

    [[nodiscard]] std::optional<Geometry2d::Point> getBallPlacementPoint() const {
        return ball_placement_point;
    }
};
