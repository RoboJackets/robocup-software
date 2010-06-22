#pragma once

#include "Team.h"

class GameState
{
public:
    enum Period
    {
        FirstHalf,
        Halftime,
        SecondHalf,
        Overtime1,
        Overtime2,
        PenaltyShootout
    };
    
    enum State
    {
        Halt,       // Robots must not move
        Stop,       // Robots must stay 500mm away from ball
        Setup,      // Robots not on starting team must stay 500mm away from ball
        Ready,      // A robot on the starting team may kick the ball
        Playing     // Normal play
    };
    
    // Types of restarts
    enum Restart
    {
        None,
        Kickoff,
        Direct,
        Indirect,
        Penalty
    };

    Period period;
    State state;
    Restart restart;
    
    // True if our team can kick the ball during a restart
    bool ourRestart;
    
    // Scores
    uint8_t ourScore;
    uint8_t theirScore;
    
    // Time in seconds remaining in the current period
    uint16_t secondsRemaining;
    
    // Counter from the most recent referee packet
    uint32_t numPackets;

    GameState()
    {
        period = FirstHalf;
        state = Halt;
        restart = None;
        ourRestart = false;
        ourScore = 0;
        theirScore = 0;
        secondsRemaining = 0;
    }
    
    ////////
    // Rule queries
    
    bool halt() const
    {
        return state == Halt;
    }
    
	bool stopped() const
	{
		return state == Stop;
	}
	
	bool playing() const
	{
		return state == Playing;
	}
	
	bool kickoff() const
    {
        return restart == Kickoff;
    }
    
    bool penalty() const
    {
        return restart == Penalty;
    }
    
    bool direct() const
    {
        return restart == Direct;
    }
    
    bool indirect() const
    {
        return restart == Indirect;
    }
    
    bool ourKickoff() const
    {
        return kickoff() && ourRestart;
    }
    
    bool ourPenalty() const
    {
        return penalty() && ourRestart;
    }
    
    bool ourDirect() const
    {
        return direct() && ourRestart;
    }
    
    bool ourIndirect() const
    {
        return indirect() && ourRestart;
    }
    
    bool ourFreeKick() const
    {
        return ourDirect() || ourIndirect();
    }
    
	bool theirKickoff() const
	{
		return kickoff() && !ourRestart;
	}
	
	bool theirPenalty() const
	{
		return penalty() && !ourRestart;
	}
	
	bool theirDirect() const
	{
		return direct() && !ourRestart;
	}
	
	bool theirIndirect() const
	{
		return indirect() && !ourRestart;
	}
	
	bool theirFreeKick() const
	{
		return theirDirect() || theirIndirect();
	}
	
	// Robots must be in position for a restart
    bool setupRestart() const
    {
        return state == Setup || state == Ready;
    }
    
    // One of our robots can kick the ball
    bool canKick() const
    {
        return state == Playing || (ourRestart && state == Playing);
    }
    
    // Our robots must stay 500mm away from the ball
    bool stayAwayFromBall() const
    {
        return state != Playing && !ourRestart;
    }
    
    // Our robots must stay on our half of the field
    bool stayOnSide() const
    {
        return setupRestart() && restart == Kickoff;
    }
    
    // Our robots (except the penalty kicker) must stay 400mm behind the penalty line
    bool stayBehindPenaltyLine() const
    {
        return restart == Penalty;
    }
};
