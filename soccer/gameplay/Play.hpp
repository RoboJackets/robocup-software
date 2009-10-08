#pragma once

#include <QString>
#include <AutoName.hpp>

#include "GameplayModule.hpp"

namespace Gameplay
{
	class GameplayModule;

	class Play: public AutoName
	{
	public:
		Play(GameplayModule *gameplay);
		virtual ~Play();
		
		GameplayModule *gameplay() const
		{
			return _gameplay;
		}
		
		// Returns true iff this play is allowed to be selected given the current state of the game.
		// The default implementation always returns true.
		virtual bool applicable();
		
		// Returns a score used to compare this play against all other applicable plays when a new play
		// is to be selected.
		//
		// The applicable play with the LOWEST score is selected.  This is intended to be convenient for error-minimizing criteria.
		// The default implementation always returns zero.
		virtual float score();
		
		// Called before run() when this play becomes current.
		virtual void start();
		
		// Runs the play for one frame.
		// Returns true if the play can continue running or false if another play should be selected.
		virtual bool run() = 0;
		
		// Called after the last run() before a different play becomes current.
		virtual void stop();
		
	protected:
		GameplayModule *_gameplay;
		
		// Convenience functions
		const GameState &gameState() const
		{
			return _gameplay->state()->gameState;
		}
		
		Gameplay::Robot *self(int i) const
		{
			return _gameplay->self[i];
		}
		
		const Gameplay::Robot *opp(int i) const
		{
			return _gameplay->opp[i];
		}
	};
}
