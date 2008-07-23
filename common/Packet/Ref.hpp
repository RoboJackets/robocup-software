#ifndef REF_HPP
#define REF_HPP

#include "Ports.hpp"

#include <stdint.h>

namespace Packet
{
	class Ref
	{
		public:
			static const uint16_t Type = RefPort;
			
            // State of gameplay.
            typedef enum
            {
                Halt,       // No motion
                Stop,       // Moving but not playing.  Can't touch the ball.
                Setup,      // Get in positions to start.
                            // Still can't touch the ball.
							// Can approach ball
                            // Only kickoff and penalty kicks use this state.
                OppStart,   // Waiting for opponent to start.
                Running     // Normal gameplay/Ok to run fully
            } State;
            
            // How gameplay has started or is about to start.
            typedef enum
            {
                None,       //regular start...occurs on force start
                Kickoff,
                Penalty,    //needs Running to begin
                Direct,     //also called freekick in sslrefbox 
                Indirect
            } Start;
    
	    typedef enum
	    {
			FirstHalf,
			HalfTime,
			SecondHalf,
			Overtime1,
			Overtime2,
			PenaltyShootout
	    } Period;
	    
	    Ref()
	    {
			goalsSelf = 0;
			goalsOpp = 0;
			counter = 0;
			
			state = Halt;
			start = None;
			period = FirstHalf;
			
			timestamp = 0;
	    }
	    
	    // state of the ref system for the team
	    State state;
	    
	    //how the team should start
	    Start start;
	    
	    // game period
	    Period period;

	    // True if our team should touch the ball to begin play.
	    bool ourStart;
	    
	    unsigned char goalsSelf;
	    unsigned char goalsOpp;
	    
	    /** vision timestamp */
	    uint64_t timestamp;
	    
	    /** ref command counter */
	    uint8_t counter;
	};
}

#endif /* REF_HPP */

