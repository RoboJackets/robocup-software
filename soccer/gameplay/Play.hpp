#pragma once

#include "Behavior.hpp"

namespace Gameplay
{
	class GameplayModule;

	class Play: public Behavior
	{
	public:
		Play(GameplayModule *gameplay);
		virtual ~Play();
		
		// Returns true iff this play is allowed to be selected given the current state of the game.
		// The default implementation always returns true.
		virtual bool applicable();
		
		// Returns a score used to compare this play against all other applicable plays when a new play
		// is to be selected.
		//
		// The applicable play with the LOWEST score is selected.  This is intended to be convenient for error-minimizing criteria.
		// The default implementation always returns zero.
		virtual float score();
	};
}
