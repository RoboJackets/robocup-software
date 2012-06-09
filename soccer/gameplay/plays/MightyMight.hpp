#pragma once

#include "../Play.hpp"

#include <gameplay/behaviors/positions/Fullback.hpp>
#include <gameplay/behaviors/Kick.hpp>

namespace Gameplay
{
	namespace Plays
	{
		/** This play just has two dumb robots that chase the ball
		 * Basic tie-breaking occurs to keep the robots from colliding too much
		 */
		class MightyMight: public Play
		{
			public:
				static void createConfiguration(Configuration *cfg);

				MightyMight(GameplayModule *gameplay);
				
				static float score(GameplayModule *gameplay);
				virtual bool run();
				
			protected:
				bool _usingKicker1;

				Behaviors::Fullback _leftFullback, _rightFullback, _centerFullback;
				Behaviors::Kick _kicker1, _kicker2;
		};
	}
}
