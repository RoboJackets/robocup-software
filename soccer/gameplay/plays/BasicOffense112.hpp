#pragma once

#include "../Play.hpp"

#include <gameplay/behaviors/positions/Fullback.hpp>
#include <gameplay/behaviors/Kick.hpp>
#include <gameplay/behaviors/Mark.hpp>

namespace Gameplay
{
	namespace Plays
	{
		/**
		 * Offense with one main striker and a supporting robot that marks nearby robots
		 * Acts as 1-1-2 field play (one forward, one midfield, two defenders)
		 */
		class BasicOffense112: public Play
		{
			public:
				BasicOffense112(GameplayModule *gameplay);
				
				static float score(GameplayModule *gameplay);
				virtual bool run();
				
			protected:
				Behaviors::Fullback _leftFullback, _rightFullback;
				Behaviors::Kick _striker;
				Behaviors::Mark _support;
		};
	}
}
