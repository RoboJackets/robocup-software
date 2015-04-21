#pragma once

#include "../Play.hpp"

#include <gameplay/behaviors/positions/Defender.hpp>
#include <gameplay/behaviors/Kick.hpp>
#include <gameplay/behaviors/Mark.hpp>

namespace Gameplay
{
	namespace Plays
	{
		/**
		 * Simple play to handle the case where the opponent team iss
		 * not present or has only a single robot playing goalie
		 *
		 * Includes a simple kicker, as well as a defense to handle cases where the ball bounces
		 *
		 * Only needs 4 robots
		 */
		class EmptyGoal: public Play
		{
			public:
				static void createConfiguration(Configuration *cfg);

				EmptyGoal(GameplayModule *gameplay);
				
				static float score(GameplayModule *gameplay);
				virtual bool run();
				
			protected:
				Behaviors::Defender _leftDefender, _rightDefender;
				Behaviors::Kick _striker;

				static ConfigDouble *_scale_speed;
				static ConfigDouble *_scale_acc;
				static ConfigDouble *_scale_w;
		};
	}
}
