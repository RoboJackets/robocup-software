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
		 * Offense with one main striker, two supporting robots, and a single defender
		 */
		class BasicOffense121: public Play
		{
			public:
				static void createConfiguration(Configuration *cfg);

				BasicOffense121(GameplayModule *gameplay);
				
				static float score(GameplayModule *gameplay);
				virtual bool run();
				
			protected:
				Behaviors::Fullback _fullback;
				Behaviors::Kick _striker;
				Behaviors::Mark _support1, _support2;
		};
	}
}
