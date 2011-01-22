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
		 * Offense with one main striker and either one or two supporting robots,
		 * with switching from defense
		 */
		class AggressiveOpen: public Play
		{
			public:
				AggressiveOpen(GameplayModule *gameplay);
				
				static float score(GameplayModule *gameplay);
				virtual bool run();
				
			protected:
				Behaviors::Fullback _leftFullback, _rightFullback;
				Behaviors::Kick _striker;
				Behaviors::Mark _supportPrimary, _supportSecondary;
		};
	}
}
