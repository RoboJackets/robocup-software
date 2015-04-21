#pragma once

#include "../Play.hpp"

#include <gameplay/behaviors/positions/Defender.hpp>
#include <gameplay/behaviors/Yank.hpp>
#include <gameplay/behaviors/Fling.hpp>
#include <gameplay/behaviors/Mark.hpp>
#include <gameplay/behaviors/Bump.hpp>

namespace Gameplay
{
	namespace Plays
	{
		/**
		 * Offense with two defenders, a supporting robot, and
		 * a single striker doing ball tricks
		 */
		class YankOpen: public Play
		{
			public:
				static void createConfiguration(Configuration *cfg);

				YankOpen(GameplayModule *gameplay);
				
				static float score(GameplayModule *gameplay);
				virtual bool run();
				
			protected:
				Behaviors::Defender _leftDefender, _rightDefender;
				Behaviors::Mark _support;

				// behaviors for the striker - switch between these
				Behaviors::Bump _strikerBump;
				Behaviors::Fling _strikerFling;
				Behaviors::Yank _strikerYank;

				static ConfigBool *_useYank;
				static ConfigDouble *_offense_hysteresis;  // determines when to switch offense players
				static ConfigDouble *_support_backoff_thresh;
				static ConfigDouble *_mark_hysteresis_coeff;
				static ConfigDouble *_support_avoid_teammate_radius;
				static ConfigDouble *_support_avoid_shot;
				static ConfigDouble *_offense_support_ratio;
				static ConfigDouble *_defense_support_ratio;

		};
	}
}
