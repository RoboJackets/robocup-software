#pragma once

#include <framework/SystemState.hpp>
#include <Team.h>
#include <Referee.hpp>

namespace RefereeCommands
{
    ////////////////////////
    // General gameplay
    
    // All robots must stop moving immediately.
    static const char Halt = 'H';
    
    // Stops gameplay.  Robots may still move, but must stay 500mm away from ball.
    static const char Stop = 'S';
    
    // Switches to normal gameplay.  May follow a Stop or restart command.
    static const char ForceStart = 's';
    
    // Allows a robot on the starting team to kick the ball.
    static const char Ready = ' ';
    
    ////////////////////////
    // Timing
    
    static const char FirstHalf = '1';
    static const char Halftime = 'h';
    static const char SecondHalf = '2';
    static const char Overtime1 = 'o';
    static const char Overtime2 = 'O';
    static const char PenaltyShootout = 'a';
    
    ////////////////////////
    // Timeouts
    //
    // A timeout can end with Stop (ended by team) or TimeoutEnd (out of time).
    
    static const char TimeoutYellow = 't';
    static const char TimeoutBlue = 'T';
    
    // Ran out of time in the timeout.
    static const char TimeoutEnd = 'z';
    
    // Ends the current timeout and resets the timeout time to its value at the beginning of the timeout.
    static const char TimeoutCancel = 'c';
    
    ////////////////////////
    // Goals scored
    
    static const char GoalYellow = 'g';
    static const char GoalBlue = 'G';
    
    static const char SubtractGoalYellow = 'd';
    static const char SubtractGoalBlue = 'D';
    
    ////////////////////////
    // Yellow/red cards
    
    static const char YellowCardYellow = 'y';
    static const char YellowCardBlue = 'Y';
    
    static const char RedCardYellow = 'r';
    static const char RedCardBlue = 'R';
    
    ////////////////////////
    // Restarts
    
    static const char KickoffYellow = 'k';
    static const char KickoffBlue = 'K';
    
    static const char PenaltyYellow = 'p';
    static const char PenaltyBlue = 'P';
    
    static const char DirectYellow = 'f';
    static const char DirectBlue = 'F';

    static const char IndirectYellow = 'i';
    static const char IndirectBlue = 'I';
};

extern const char *RefereeAddress;
static const int RefereePort = 10001;

class RefereeHandler
{
public:
    // Distance in meters that the ball must travel for a kick to be detected
    static const float KickThreshold = 0.050f;
    
    RefereeHandler(SystemState *state);
    
    // Called periodically.  Checks vision data for ball movement.
    void run();
    
    // Handles a packet from the referee.
    // If force == true, the packet's counter value is ignored.
    void packet(const Packet::Referee *data);
    void command(uint8_t command);

    ////////
    // Queries
    
    bool halt() const
    {
        return _state->gameState.state == SystemState::GameState::Halt;
    }
    
    // The most recent restart was for our team
    bool ourRestart() const
    {
        return _state->gameState.startingTeam == _state->team;
    }
    
    bool kickoff() const
    {
        return _state->gameState.restart == SystemState::GameState::Kickoff;
    }
    
    bool penalty() const
    {
        return _state->gameState.restart == SystemState::GameState::Penalty;
    }
    
    bool direct() const
    {
        return _state->gameState.restart == SystemState::GameState::Direct;
    }
    
    bool indirect() const
    {
        return _state->gameState.restart == SystemState::GameState::Indirect;
    }
    
    bool ourKickoff() const
    {
        return kickoff() && ourRestart();
    }
    
    bool ourPenalty() const
    {
        return kickoff() && ourPenalty();
    }
    
    bool ourDirect() const
    {
        return kickoff() && ourDirect();
    }
    
    bool ourIndirect() const
    {
        return kickoff() && ourIndirect();
    }
    
    bool ourFreeKick() const
    {
        return ourDirect() || ourIndirect();
    }
    
    // Robots must be in position for a restart
    bool setupRestart() const
    {
        return _state->gameState.state == SystemState::GameState::Start ||
               _state->gameState.state == SystemState::GameState::Ready;
    }
    
    // One of our robots can kick the ball
    bool canKick() const
    {
        return _state->gameState.state == SystemState::GameState::Playing ||
               (ourRestart() && _state->gameState.state == SystemState::GameState::Playing);
    }
    
    // Our robots must stay 500mm away from the ball
    bool stayAwayFromBall() const
    {
        return _state->gameState.state != SystemState::GameState::Playing &&
               _state->gameState.startingTeam != _state->team;
    }
    
    // Our robots must stay on our half of the field
    bool stayOnSide() const
    {
        return setupRestart() && _state->gameState.restart == SystemState::GameState::Kickoff;
    }
    
    // Our robots (except the penalty kicker) must stay 400mm behind the penalty line
    bool stayBehindPenaltyLine() const
    {
        return _state->gameState.restart == SystemState::GameState::Penalty;
    }

    // True if the ball has been kicked since the last restart began
    bool kicked() const
    {
        return _kicked;
    }

protected:
    // Last counter value received or -1 if no packets have been processed
    int _counter;
    
    SystemState *_state;
    
    bool _lastBallValid;
    bool _kicked;
    Geometry::Point2d _lastBallPos;
};
