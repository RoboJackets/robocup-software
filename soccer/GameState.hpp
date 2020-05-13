#pragma once

#include <Geometry2d/Point.hpp>
#include <Geometry2d/TransformMatrix.hpp>
#include <time.hpp>

#include "Constants.hpp"
#include "RefereeEnums.hpp"
#include "TeamInfo.hpp"

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

    Period period;
    State state;
    Restart restart;

    // True if our team can kick the ball during a restart
    bool ourRestart;

    // Scores
    int ourScore;
    int theirScore;

    // Time in seconds remaining in the current period
    RJ::Seconds stage_time_left;

    TeamInfo OurInfo;
    TeamInfo TheirInfo;

    // Bool representing if we are the blue team
    bool blueTeam;

    Geometry2d::Point ballPlacementPoint;

    RefereeModuleEnums::Stage raw_stage =
        RefereeModuleEnums::NORMAL_FIRST_HALF_PRE;
    RefereeModuleEnums::Command raw_command = RefereeModuleEnums::Command::HALT;

    GameState()
        : period{FirstHalf},
          state{Halt},
          restart{None},
          ourRestart{false},
          ourScore{0},
          theirScore{0},
          OurInfo{},
          TheirInfo{},
          blueTeam{false},
          ballPlacementPoint{} {}

    GameState(Period period, State state, Restart restart, bool ourRestart,
              int ourScore, int theirScore, RJ::Seconds stage_time_left,
              TeamInfo our_info, TeamInfo their_info, bool blueTeam,
              Geometry2d::Point ballPlacementPoint,
              RefereeModuleEnums::Stage stage,
              RefereeModuleEnums::Command command)
        : period{period},
          state{state},
          restart{restart},
          ourRestart{ourRestart},
          ourScore{ourScore},
          theirScore{theirScore},
          stage_time_left{stage_time_left},
          OurInfo{our_info},
          TheirInfo{their_info},
          blueTeam{blueTeam},
          ballPlacementPoint{ballPlacementPoint},
          raw_stage{stage},
          raw_command{command} {}

    ////////
    // Rule queries

    bool isFirstHalf() const { return period == FirstHalf; }

    bool isSecondHalf() const { return period == SecondHalf; }

    bool isHalftime() const { return period == Halftime; }

    bool isOvertime1() const { return period == Overtime1; }

    bool isOvertime2() const { return period == Overtime2; }

    bool isPenaltyShootout() const { return period == PenaltyShootout; }

    bool halt() const { return state == Halt; }

    bool stopped() const { return state == Stop; }

    bool playing() const { return state == Playing; }

    bool kickoff() const { return restart == Kickoff; }

    bool penalty() const { return restart == Penalty; }

    bool placement() const { return restart == Placement; }

    bool isOurRestart() const { return ourRestart; }

    bool direct() const { return restart == Direct; }

    bool indirect() const { return restart == Indirect; }

    bool ourKickoff() const { return kickoff() && ourRestart; }

    bool ourPenalty() const { return penalty() && ourRestart; }

    bool ourDirect() const { return direct() && ourRestart; }

    bool ourIndirect() const { return indirect() && ourRestart; }

    bool ourFreeKick() const { return ourDirect() || ourIndirect(); }

    bool ourPlacement() const { return placement() && ourRestart; }

    bool theirKickoff() const { return kickoff() && !ourRestart; }

    bool theirPenalty() const { return penalty() && !ourRestart; }

    bool theirDirect() const { return direct() && !ourRestart; }

    bool theirIndirect() const { return indirect() && !ourRestart; }

    bool theirFreeKick() const { return theirDirect() || theirIndirect(); }

    bool theirPlacement() const { return placement() && !ourRestart; }

    // Robots must be in position for a restart
    bool setupRestart() const { return state == Setup || state == Ready; }

    bool inSetupState() const { return state == Setup; }

    bool inReadyState() const { return state == Ready; }

    // One of our robots can kick the ball
    bool canKick() const {
        return state == Playing || (ourRestart && state == Playing);
    }

    // Our robots must stay 500mm away from the ball
    bool stayAwayFromBall() const { return state != Playing && !ourRestart; }

    // Our robots must stay on our half of the field
    bool stayOnSide() const { return setupRestart() && restart == Kickoff; }

    // Our robots (except the penalty kicker) must stay 400mm behind the penalty
    // line
    bool stayBehindPenaltyLine() const { return restart == Penalty; }

    void setBallPlacementPoint(float x, float y) {
        Geometry2d::TransformMatrix _worldToTeam =
            Geometry2d::TransformMatrix();
        _worldToTeam *= Geometry2d::TransformMatrix::translate(
            0, Field_Dimensions::Current_Dimensions.Length() / 2.0f);
        ballPlacementPoint =
            _worldToTeam * Geometry2d::Point(x / 1000, y / 1000);
    }

    static Geometry2d::Point convertToBallPlacementPoint(float x, float y) {
        Geometry2d::TransformMatrix world_to_team =
            Geometry2d::TransformMatrix();
        world_to_team *= Geometry2d::TransformMatrix::translate(
            0, Field_Dimensions::Current_Dimensions.Length() / 2.0f);

        return world_to_team * Geometry2d::Point(x / 1000, y / 1000);
    }

    Geometry2d::Point getBallPlacementPoint() const {
        return ballPlacementPoint;
    }

    uint getGoalieId() const { return OurInfo.goalie; }
};
